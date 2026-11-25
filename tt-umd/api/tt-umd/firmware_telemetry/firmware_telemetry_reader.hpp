// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <map>

#include "tt-umd/types/noc_id.hpp"

namespace tt::umd {
/**
 * @brief Raw telemetry reader interface.
 */
class FirmwareTelemetryReader {
public:
    virtual ~FirmwareTelemetryReader() = default;

    /**
     * @brief Reads a telemetry entry by tag.
     * @param tag Telemetry tag identifying the entry to read.
     * @return uint32_t The telemetry value.
     */
    virtual uint32_t read_entry(uint8_t tag, [[maybe_unused]] NocId noc_id = NocId::DEFAULT_NOC) = 0;

    /**
     * @brief Checks whether a telemetry entry is available.
     * @param tag Telemetry tag identifying the entry to check.
     * @param noc_id NOC to route through.
     * @return true if the entry exists in the firmware telemetry table.
     */
    virtual bool is_entry_available(uint8_t tag, [[maybe_unused]] NocId noc_id = NocId::DEFAULT_NOC) = 0;

    /**
     * @brief Reads every available telemetry entry in one coherent device read.
     * @return Map of telemetry tag to value. A single read gives a consistent snapshot and live
     *         values even for entries the reader caches as static, which repeated read_entry calls
     *         do not. Prefer this over per-tag reads when sampling many tags or sampling fast.
     */
    virtual std::map<uint8_t, uint32_t> read_all_entries([[maybe_unused]] NocId noc_id = NocId::DEFAULT_NOC) = 0;
};
}  // namespace tt::umd
