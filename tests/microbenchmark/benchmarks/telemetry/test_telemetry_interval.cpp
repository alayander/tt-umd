// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tt-umd/arch/blackhole_implementation.hpp"
#include "tt-umd/pcie/pci_device.hpp"
#include "tt-umd/tt_device/firmware/device_firmware.hpp"
#include "tt-umd/tt_device/protocol/device_protocol.hpp"
#include "tt-umd/tt_device/tt_device.hpp"
#include "tt-umd/tt_device/tt_device_factory.hpp"
#include "tt-umd/types/noc_id.hpp"
#include "tt-umd/types/register_access_width.hpp"
#include "tt-umd/types/telemetry.hpp"
#include "tt-umd/types/xy_pair.hpp"
#include "tt-umd/utils/timeouts.hpp"

using namespace tt::umd;

namespace {

// Word 0 of the ARC request: low byte is the command, byte 1 is the submessage ID. The firmware
// reads the submessage ID from byte 1, so both have to be packed into the message code - which is
// exactly what the uint32_t send_device_command path preserves.
constexpr uint32_t TT_SMC_MSG_CHARACTERISATION = 0xC6;
constexpr uint32_t TT_SUB_MSG_SET_TELEMETRY_UPDATE_INTERVAL = 0x5;
constexpr uint32_t SET_TELEMETRY_UPDATE_INTERVAL_MSG =
    TT_SMC_MSG_CHARACTERISATION | (TT_SUB_MSG_SET_TELEMETRY_UPDATE_INTERVAL << 8);

// --- TT_SMC_MSG_COUNTER: generic counter bank access (see tenstorrent/msgqueue.h counter_rqst) ---
// Request layout, byte-packed into words: word0 = command_code; word1 = command | bank<<8 |
// index<<16 | reserved<<24; word2 = mask. Response: return_values[0] = status word (data[1]),
// return_values[1] = counter value (data[2]).
constexpr uint32_t TT_SMC_MSG_COUNTER = 0x35;
constexpr uint32_t COUNTER_CMD_GET = 0;
constexpr uint32_t COUNTER_CMD_CLEAR = 1;
constexpr uint32_t COUNTER_BANK_DVFS = 1;

// Only the three original DVFS counters; this test targets firmware without the period histogram.
constexpr int DVFS_COUNTER_COUNT = 3;
const std::array<const char*, DVFS_COUNTER_COUNT> DVFS_COUNTER_LABELS = {
    "dropped_ticks",
    "max_period_us",
    "max_pass_us",
};

// Interval the firmware restores to; 100ms is the firmware's default refresh cadence.
constexpr uint32_t DEFAULT_TELEM_INTERVAL_MS = 100;

// Knob under test. 3ms is well below the default, which is what makes the DVFS loop's work queue
// contend with telemetry refresh. Override with TT_TELEM_INTERVAL_MS to sweep other values.
constexpr uint32_t DEFAULT_KNOB_TELEM_INTERVAL_MS = 3;

// Default sampling window; override with TT_SAMPLE_DURATION_S.
constexpr double DEFAULT_SAMPLE_DURATION_S = 2.0;

// Telemetry tag id -> name (tt-umd/types/telemetry.hpp / tt-system-firmware telemetry.h). Only used
// to label the JSON columns; the on-device table is the source of truth for which tags exist.
std::string tag_id_to_name(uint16_t id) {
    static const std::map<uint16_t, const char*> names = {
        {1, "BOARD_ID_HIGH"},     {2, "BOARD_ID_LOW"},      {3, "ASIC_ID"},
        {4, "HARVESTING_STATE"},  {5, "UPDATE_TELEM_SPEED"},{6, "VCORE"},
        {7, "TDP"},               {8, "TDC"},               {9, "VDD_LIMITS"},
        {10, "THM_LIMIT_SHUTDOWN"},{11, "ASIC_TEMPERATURE"},{12, "VREG_TEMPERATURE"},
        {13, "BOARD_TEMPERATURE"},{14, "AICLK"},            {15, "AXICLK"},
        {16, "ARCCLK"},           {31, "FAN_SPEED"},        {32, "TIMER_HEARTBEAT"},
        {33, "TELEM_ENUM_COUNT"}, {39, "INPUT_CURRENT"},    {41, "FAN_RPM"},
        {51, "MAX_GDDR_TEMP"},    {53, "BOARD_POWER_LIMIT"},{54, "INPUT_POWER"},
        {55, "TDC_LIMIT_MAX"},    {56, "THM_LIMIT_THROTTLE"},{63, "AICLK_LIMIT_MAX"},
        {64, "TDP_LIMIT_MAX"},    {65, "AICLK_ARB_MIN"},    {66, "AICLK_ARB_MAX"},
        {75, "KERNEL_THROTTLER"},
    };
    auto it = names.find(id);
    return it != names.end() ? it->second : "TAG_" + std::to_string(id);
}

// Reverse lookup of tag_id_to_name for parsing a tag name; 0 means "not a known name".
uint16_t tag_id_from_name(const std::string& upper_name) {
    for (uint16_t id = 1; id < 256; id++) {
        if (tag_id_to_name(id) == upper_name) {
            return id;
        }
    }
    return 0;
}

// Default JSON columns; override with TT_TELEM_TAGS (comma list of names/ids, or "all").
constexpr const char* DEFAULT_TELEM_TAGS = "TIMER_HEARTBEAT,VCORE,TDP,TDC,AICLK,ASIC_TEMPERATURE,INPUT_POWER";

uint32_t env_u32(const char* name, uint32_t fallback) {
    const char* env = std::getenv(name);
    if (env != nullptr) {
        const long parsed = std::strtol(env, nullptr, 10);
        if (parsed > 0) {
            return static_cast<uint32_t>(parsed);
        }
    }
    return fallback;
}

double env_double(const char* name, double fallback) {
    const char* env = std::getenv(name);
    if (env != nullptr) {
        const double parsed = std::strtod(env, nullptr);
        if (parsed > 0.0) {
            return parsed;
        }
    }
    return fallback;
}

// Reads the firmware telemetry table straight off the ARC core, the same way
// BlackholeArcTelemetryReader does: tag-table address from SCRATCH_RAM_13 and values address from
// SCRATCH_RAM_12 (both over the ARC XBAR), the tag->offset map parsed once at construction. Samples
// are a single block read of the whole values array - one coherent snapshot, no per-sample re-init.
class TelemTable {
public:
    TelemTable(DeviceProtocol* proto, tt_xy_pair arc_core, NocId noc) : proto_(proto), core_(arc_core), noc_(noc) {
        // Wait for ARC to publish the table pointer (non-zero SCRATCH_RAM_13), then parse once.
        uint32_t table_addr = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (table_addr == 0 && std::chrono::steady_clock::now() < deadline) {
            table_addr = read_ctrl32(blackhole::ARC_NOC_XBAR_ADDRESS_START + blackhole::SCRATCH_RAM_13);
            if (table_addr == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        values_addr_ = read_ctrl32(blackhole::ARC_NOC_XBAR_ADDRESS_START + blackhole::SCRATCH_RAM_12);

        proto_->read_data(&entry_count_, core_, static_cast<uint64_t>(table_addr) + sizeof(uint32_t),
                          sizeof(uint32_t), noc_);
        if (entry_count_ == 0 || entry_count_ >= 1024) {
            throw std::runtime_error("telemetry table has an implausible entry count (ARC not up?)");
        }

        // Tag table: one u32 per entry, low 16 = tag, high 16 = offset into the values array.
        std::vector<uint32_t> tag_table(entry_count_);
        proto_->read_data(tag_table.data(), core_, static_cast<uint64_t>(table_addr) + 2 * sizeof(uint32_t),
                          entry_count_ * sizeof(uint32_t), noc_);
        for (uint32_t e : tag_table) {
            const uint16_t tag = e & 0xFFFF;
            const uint16_t offset = e >> 16;
            tags_.push_back(tag);
            tag_to_offset_[tag] = offset;
            if (tag == TelemetryTag::TIMER_HEARTBEAT) {
                hb_offset_ = offset;
            }
            if (tag == TelemetryTag::UPDATE_TELEM_SPEED) {
                update_speed_offset_ = offset;
            }
        }
    }

    // Tags present in the table, in table order.
    const std::vector<uint16_t>& tags() const { return tags_; }

    bool has_tag(uint16_t tag) const { return tag_to_offset_.count(tag) != 0; }

    // Index of a tag's value within a full snapshot (see read_snapshot).
    uint16_t offset_for(uint16_t tag) const {
        auto it = tag_to_offset_.find(tag);
        return it != tag_to_offset_.end() ? it->second : 0;
    }

    uint32_t entry_count() const { return entry_count_; }

    uint32_t read_heartbeat() { return read_value_word(hb_offset_); }

    uint32_t read_update_speed() { return read_value_word(update_speed_offset_); }

    // One block read of the whole values array into out, indexed by offset (offset_for()).
    void read_snapshot(std::vector<uint32_t>& out) {
        out.resize(entry_count_);
        proto_->read_data(out.data(), core_, values_addr_, entry_count_ * sizeof(uint32_t), noc_);
    }

private:
    uint32_t read_ctrl32(uint64_t addr) {
        uint32_t v = 0;
        proto_->read_ctrl(&v, core_, addr, sizeof(uint32_t), noc_, RegisterAccessWidth::BITS32);
        return v;
    }

    uint32_t read_value_word(uint16_t offset) {
        uint32_t v = 0;
        proto_->read_data(&v, core_, values_addr_ + static_cast<uint64_t>(offset) * sizeof(uint32_t), sizeof(uint32_t),
                          noc_);
        return v;
    }

    DeviceProtocol* proto_;
    tt_xy_pair core_;
    NocId noc_;
    uint64_t values_addr_ = 0;
    uint32_t entry_count_ = 0;
    std::vector<uint16_t> tags_;
    std::map<uint16_t, uint16_t> tag_to_offset_;
    uint16_t hb_offset_ = 0;
    uint16_t update_speed_offset_ = 0;
};

// The subset of tags to accumulate/emit: names for the JSON header and their offsets into the full
// snapshot. The full table is still read every sample; this only picks which columns we keep.
struct TagSelection {
    std::vector<std::string> names;
    std::vector<uint16_t> offsets;
};

TagSelection resolve_tags(const TelemTable& tbl, const std::string& spec) {
    TagSelection sel;
    const auto add = [&](uint16_t id) {
        if (!tbl.has_tag(id)) {
            std::cerr << "[  TELEM   ] tag " << tag_id_to_name(id) << " not in this FW's table, skipping\n";
            return;
        }
        sel.names.push_back(tag_id_to_name(id));
        sel.offsets.push_back(tbl.offset_for(id));
    };

    if (spec == "all") {
        for (uint16_t tag : tbl.tags()) {
            add(tag);
        }
        return sel;
    }

    size_t start = 0;
    while (start <= spec.size()) {
        const size_t comma = spec.find(',', start);
        std::string token = spec.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        start = comma == std::string::npos ? spec.size() + 1 : comma + 1;
        token.erase(0, token.find_first_not_of(" \t"));
        if (!token.empty()) {
            token.erase(token.find_last_not_of(" \t") + 1);
        }
        if (token.empty()) {
            continue;
        }
        std::transform(token.begin(), token.end(), token.begin(), ::toupper);
        uint16_t id = 0;
        if (std::isdigit(static_cast<unsigned char>(token[0]))) {
            id = static_cast<uint16_t>(std::strtol(token.c_str(), nullptr, 10));
        } else {
            id = tag_id_from_name(token);
            if (id == 0) {
                std::cerr << "[  TELEM   ] unknown tag '" << token << "', skipping\n";
                continue;
            }
        }
        add(id);
    }
    return sel;
}

uint32_t set_telem_interval(TTDevice* tt_device, uint32_t interval_ms) {
    DeviceCommandResult result = tt_device->get_device_firmware()->send_device_command(
        SET_TELEMETRY_UPDATE_INTERVAL_MSG, {interval_ms}, timeout::ARC_MESSAGE_TIMEOUT);
    return result.exit_code;
}

// Confirms the firmware applied the interval by reading UPDATE_TELEM_SPEED live off the table (the
// firmware writes it synchronously in the set handler; poll briefly to stay robust).
uint32_t read_applied_interval_ms(TelemTable& tbl, uint32_t expected_ms) {
    uint32_t value = 0;
    for (int i = 0; i < 20; i++) {
        value = tbl.read_update_speed();
        if (value == expected_ms) {
            return value;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return value;
}

uint32_t read_dvfs_counter(TTDevice* tt_device, uint32_t index) {
    const uint32_t word1 = COUNTER_CMD_GET | (COUNTER_BANK_DVFS << 8) | (index << 16);
    DeviceCommandResult result =
        tt_device->get_device_firmware()->send_device_command(TT_SMC_MSG_COUNTER, {word1, 0}, timeout::ARC_MESSAGE_TIMEOUT);
    // return_values[0] is the status word; return_values[1] is the counter value (data[2]).
    return result.return_values.size() >= 2 ? result.return_values[1] : 0;
}

std::array<uint32_t, DVFS_COUNTER_COUNT> read_all_dvfs_counters(TTDevice* tt_device) {
    std::array<uint32_t, DVFS_COUNTER_COUNT> values{};
    for (int i = 0; i < DVFS_COUNTER_COUNT; i++) {
        values[i] = read_dvfs_counter(tt_device, static_cast<uint32_t>(i));
    }
    return values;
}

void clear_dvfs_counters(TTDevice* tt_device) {
    const uint32_t word1 = COUNTER_CMD_CLEAR | (COUNTER_BANK_DVFS << 8);
    const uint32_t mask = (1u << DVFS_COUNTER_COUNT) - 1;  // clear the three DVFS counters
    tt_device->get_device_firmware()->send_device_command(TT_SMC_MSG_COUNTER, {word1, mask}, timeout::ARC_MESSAGE_TIMEOUT);
}

// Host wall-clock time in nanoseconds since the Unix epoch, recorded per accumulated sample.
uint64_t host_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// Accumulated results for one chip; serialized to JSON at the very end.
struct ChipResult {
    int chip = 0;
    uint32_t poll_us = 0;     // host poll period (0 = flat-out)
    uint64_t sweeps = 0;      // host polls of TIMER_HEARTBEAT
    uint64_t fw_updates = 0;  // heartbeat changes (= new FW-published samples)
    double duration_s = 0.0;
    std::array<uint32_t, DVFS_COUNTER_COUNT> counters{};
    std::vector<std::string> columns;                 // tag names, table order
    std::vector<uint64_t> sample_host_ns;             // host wall-clock timestamp per accumulated sample
    std::vector<std::vector<uint32_t>> sample_values; // selected tag values per accumulated sample
};

// Samples for duration_s: polls the heartbeat (one word) to detect a new FW sample, and on each
// change does one full-table block read, accumulating the whole table in memory along with a host
// wall-clock timestamp. poll_us throttles the poll cadence (0 = flat-out); it should stay well below
// the publish interval so every update is still caught - we default it to interval/3 at the call site.
void sample_run(TelemTable& tbl, double duration_s, uint32_t poll_us, const std::vector<uint16_t>& sel_offsets,
                ChipResult& out) {
    out.poll_us = poll_us;
    const uint16_t hb_offset = tbl.offset_for(TelemetryTag::TIMER_HEARTBEAT);
    uint32_t last_hb = tbl.read_heartbeat();

    const auto start = std::chrono::steady_clock::now();
    const auto end = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                 std::chrono::duration<double>(duration_s));
    auto now = start;
    std::vector<uint32_t> snapshot;
    while (now < end) {
        const auto iter_start = std::chrono::steady_clock::now();
        const uint32_t hb = tbl.read_heartbeat();
        out.sweeps++;
        now = std::chrono::steady_clock::now();
        if (hb != last_hb) {
            out.fw_updates++;

            // Record the host wall-clock time at the moment we detected this new sample.
            out.sample_host_ns.push_back(host_time_ns());

            // Still read the whole table coherently, but keep only the requested columns.
            tbl.read_snapshot(snapshot);
            std::vector<uint32_t> row;
            row.reserve(sel_offsets.size());
            for (uint16_t off : sel_offsets) {
                row.push_back(off < snapshot.size() ? snapshot[off] : 0);
            }
            out.sample_values.push_back(std::move(row));

            // Sync detection to the heartbeat in the snapshot we actually stored (it may have ticked
            // between the detect read and the block read), so stored rows stay strictly increasing.
            last_hb = snapshot[hb_offset] >= hb ? snapshot[hb_offset] : hb;
        }
        if (poll_us > 0 && now < end) {
            std::this_thread::sleep_until(iter_start + std::chrono::microseconds(poll_us));
            now = std::chrono::steady_clock::now();
        }
    }
    out.duration_s = std::chrono::duration<double>(now - start).count();
}

void log_chip(const ChipResult& ch) {
    const double sweep_hz = ch.duration_s > 0 ? ch.sweeps / ch.duration_s : 0.0;
    const double fw_hz = ch.duration_s > 0 ? ch.fw_updates / ch.duration_s : 0.0;
    std::cout << "[  TELEM   ] chip " << ch.chip << ": duration=" << ch.duration_s << "s, poll="
              << (ch.poll_us > 0 ? std::to_string(ch.poll_us) + "us" : "flat-out") << "\n";
    std::cout << "[  TELEM   ]   host sweep rate:   " << sweep_hz << " Hz (" << ch.sweeps << " polls)\n";
    std::cout << "[  TELEM   ]   FW publish rate:   " << fw_hz << " Hz (~" << (fw_hz > 0 ? 1000.0 / fw_hz : 0.0)
              << " ms), " << ch.fw_updates << " samples\n";
    std::cout << "[  TELEM   ]   DVFS counters (this run):";
    for (int i = 0; i < DVFS_COUNTER_COUNT; i++) {
        std::cout << " " << DVFS_COUNTER_LABELS[i] << "=" << ch.counters[i];
    }
    std::cout << "\n";
}

void write_json(std::ostream& os, uint32_t interval_ms, double duration_s, const std::vector<ChipResult>& chips) {
    const auto num = [](double v) {
        std::ostringstream s;
        s << std::setprecision(6) << v;
        return s.str();
    };

    os << "{\n";
    os << "  \"interval_ms\": " << interval_ms << ",\n";
    os << "  \"duration_s\": " << num(duration_s) << ",\n";
    os << "  \"chips\": [\n";
    for (size_t c = 0; c < chips.size(); c++) {
        const ChipResult& ch = chips[c];
        const double sweep_hz = ch.duration_s > 0 ? ch.sweeps / ch.duration_s : 0.0;
        const double fw_hz = ch.duration_s > 0 ? ch.fw_updates / ch.duration_s : 0.0;

        os << "    {\n";
        os << "      \"chip\": " << ch.chip << ",\n";
        os << "      \"poll_us\": " << ch.poll_us << ",\n";
        os << "      \"host_sweep_hz\": " << num(sweep_hz) << ",\n";
        os << "      \"fw_publish_hz\": " << num(fw_hz) << ",\n";
        os << "      \"fw_updates\": " << ch.fw_updates << ",\n";
        os << "      \"dvfs_counters\": {";
        for (int i = 0; i < DVFS_COUNTER_COUNT; i++) {
            os << "\"" << DVFS_COUNTER_LABELS[i] << "\": " << ch.counters[i] << (i + 1 < DVFS_COUNTER_COUNT ? ", " : "");
        }
        os << "},\n";
        os << "      \"tags\": [";
        for (size_t i = 0; i < ch.columns.size(); i++) {
            os << "\"" << ch.columns[i] << "\"" << (i + 1 < ch.columns.size() ? ", " : "");
        }
        os << "],\n";
        // Each sample is an explicit object: the host wall-clock timestamp (ns since the Unix epoch)
        // at which the host detected the new FW sample, plus the selected tag values (order matches
        // "tags" above).
        os << "      \"samples\": [";
        for (size_t s = 0; s < ch.sample_values.size(); s++) {
            os << (s == 0 ? "\n        " : ",\n        ") << "{\"host_time_ns\": "
               << (s < ch.sample_host_ns.size() ? ch.sample_host_ns[s] : 0) << ", \"values\": [";
            for (size_t i = 0; i < ch.sample_values[s].size(); i++) {
                os << (i == 0 ? "" : ", ") << ch.sample_values[s][i];
            }
            os << "]}";
        }
        os << (ch.sample_values.empty() ? "]\n" : "\n      ]\n");
        os << "    }" << (c + 1 < chips.size() ? ",\n" : "\n");
    }
    os << "  ]\n";
    os << "}\n";
}

}  // namespace

// Characterizes the telemetry update interval against the DVFS loop, entirely on the tt-umd layer
// (TTDevice + direct telemetry-table reads), so the same sequence drops into tt-metal. It runs on a
// single chip - TT_TELEM_CHIP (index into the enumerated device list), defaulting to the first chip
// (0) - and:
//   1. sets the interval to a configurable knob (default 3ms) via the PR #1448 message,
//   2. confirms the firmware applied it (live read-back of UPDATE_TELEM_SPEED from the table),
//   3. clears the DVFS counters so they count only this run,
//   4. samples: polls TIMER_HEARTBEAT and, on each change, block-reads the whole telemetry table and
//      accumulates it in memory (host timestamps),
//   5. reads the DVFS counters back (this run's dropped ticks / max period / max pass),
//   6. restores the default interval (100ms) in TearDown,
// then writes every accumulated sample to JSON once at the end (TT_TELEM_JSON).
//
// Counter deltas are logged, not asserted: the DVFS loop only falls behind under real compute load
// (e.g. run tt-burnin alongside this test), so pass/fail stays on what the harness itself controls -
// the interval was set, applied, and restored. Firmware bring-up mirrors test_arc_messages_bh.cpp.
// This test targets firmware with the three original DVFS counters (no period histogram).
//
// Env knobs: TT_TELEM_CHIP (chip index, default 0), TT_TELEM_INTERVAL_MS (knob, default 3),
// TT_SAMPLE_DURATION_S (default 2.0), TT_TELEM_JSON (path; writes accumulated samples as JSON).
class BlackholeTelemetryInterval : public ::testing::Test {
protected:
    void SetUp() override {
        std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
        if (pci_device_ids.empty()) {
            GTEST_SKIP() << "No Blackhole PCIe devices found.";
        }

        // Run on a single chip: the one passed in TT_TELEM_CHIP (index into the enumerated device
        // list), defaulting to the first chip (0). No other chips are opened or touched.
        const char* chip_env = std::getenv("TT_TELEM_CHIP");
        chip_index_ = chip_env != nullptr ? static_cast<int>(std::strtol(chip_env, nullptr, 10)) : 0;
        ASSERT_GE(chip_index_, 0) << "TT_TELEM_CHIP must be >= 0";
        ASSERT_LT(chip_index_, static_cast<int>(pci_device_ids.size()))
            << "TT_TELEM_CHIP=" << chip_index_ << " but only " << pci_device_ids.size() << " chip(s) present";

        tt_device_ = create_tt_device(pci_device_ids[chip_index_]);
        tt_device_->set_power_state(TTDevice::PowerState::BUSY);
        tt_device_->get_device_firmware()->init_firmware(timeout::ARC_STARTUP_TIMEOUT);
    }

    void TearDown() override {
        if (tt_device_ == nullptr) {
            return;
        }
        // Restore the default interval regardless of what the test body did, and confirm it took.
        TelemTable tbl(tt_device_->get_device_protocol(), tt_device_->get_arc_core(NocId::NOC0), NocId::NOC0);
        EXPECT_NO_THROW({ EXPECT_EQ(set_telem_interval(tt_device_.get(), DEFAULT_TELEM_INTERVAL_MS), 0u); });
        EXPECT_EQ(read_applied_interval_ms(tbl, DEFAULT_TELEM_INTERVAL_MS), DEFAULT_TELEM_INTERVAL_MS);
        tt_device_->set_power_state(TTDevice::PowerState::IDLE);
    }

    std::unique_ptr<TTDevice> tt_device_;
    int chip_index_ = 0;
};

TEST_F(BlackholeTelemetryInterval, SweepUpdateInterval) {
    const uint32_t knob_ms = env_u32("TT_TELEM_INTERVAL_MS", DEFAULT_KNOB_TELEM_INTERVAL_MS);
    const double duration_s = env_double("TT_SAMPLE_DURATION_S", DEFAULT_SAMPLE_DURATION_S);
    const char* json_path = std::getenv("TT_TELEM_JSON");

    // Poll at interval/3 by default so we comfortably oversample every published update without
    // spinning flat-out. TT_TELEM_POLL_US overrides it (set 0 to poll as fast as possible).
    const char* poll_env = std::getenv("TT_TELEM_POLL_US");
    const uint32_t poll_us =
        poll_env != nullptr ? static_cast<uint32_t>(std::strtoul(poll_env, nullptr, 10)) : (knob_ms * 1000) / 3;

    const std::string tag_spec = std::getenv("TT_TELEM_TAGS") ? std::getenv("TT_TELEM_TAGS") : DEFAULT_TELEM_TAGS;

    std::vector<ChipResult> results;

    {
        TTDevice* tt_device = tt_device_.get();
        TelemTable tbl(tt_device->get_device_protocol(), tt_device->get_arc_core(NocId::NOC0), NocId::NOC0);

        // 1. Set the interval knob and confirm the firmware actually applied it.
        uint32_t exit_code = 0;
        EXPECT_NO_THROW({ exit_code = set_telem_interval(tt_device, knob_ms); });
        EXPECT_EQ(exit_code, 0u);
        EXPECT_EQ(read_applied_interval_ms(tbl, knob_ms), knob_ms);

        // 2. Clear the DVFS counters so the post-run read reflects only this run window.
        clear_dvfs_counters(tt_device);

        // 3. Sample: full-table block read every heartbeat change; keep only the requested columns.
        TagSelection sel = resolve_tags(tbl, tag_spec);
        ChipResult result;
        result.chip = chip_index_;
        result.columns = sel.names;
        sample_run(tbl, duration_s, poll_us, sel.offsets, result);

        // 4. Read the counters back (this run's dropped ticks etc.).
        result.counters = read_all_dvfs_counters(tt_device);

        log_chip(result);
        results.push_back(std::move(result));
    }

    // 5. Write every accumulated sample out as JSON, once, at the end.
    if (json_path != nullptr) {
        std::ofstream json_file(json_path);
        ASSERT_TRUE(json_file.is_open()) << "could not open TT_TELEM_JSON path: " << json_path;
        write_json(json_file, knob_ms, duration_s, results);
        json_file.close();
        std::cout << "[  TELEM   ] wrote telemetry JSON to " << json_path << "\n";
    }
}
