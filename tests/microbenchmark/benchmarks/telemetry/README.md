# Fast telemetry gathering

`test_telemetry_interval.cpp` — gtest `BlackholeTelemetryInterval.SweepUpdateInterval`.

A harness for **gathering telemetry at a faster rate than the firmware's default**. Normally the
firmware refreshes its telemetry table every 100 ms; this lowers the refresh interval to a
configurable knob (default 3 ms), then streams the telemetry table off the device and accumulates
the requested tags to JSON. It runs entirely on the tt-umd layer (`TTDevice` + direct
telemetry-table reads), on a **single chip** (`TT_TELEM_CHIP`, default 0).

It also reports how fast/steadily the firmware actually publishes at the lowered interval, and reads
the DVFS timing counters for the run as a safety check that the faster cadence isn't starving the
DVFS loop. The default interval (100 ms) is restored at the end.

## What it does

1. **Raise the telemetry rate**: set the refresh interval to the knob (default 3 ms, vs the 100 ms
  default) via the characterization message: `TT_SMC_MSG_CHARACTERISATION` (`0xC6`) with
   submessage `SET_TELEMETRY_UPDATE_INTERVAL` (`0x5`) packed into word 0. This relies on the
   `uint32_t send_device_command` path that preserves the full word 0 (the submessage id lives in
   byte 1).
2. **Confirm applied** by reading `UPDATE_TELEM_SPEED` (tag 5) live via the device's
   `FirmwareTelemetryReader`.
3. **Clear** the DVFS counters so they reflect only this run.
4. **Sample**: poll `TIMER_HEARTBEAT` (one cheap word via `read_entry`); on every change do one
   coherent full-table read (`FirmwareTelemetryReader::read_all_entries`) and accumulate only the
   **requested tags** in memory, with a host timestamp.
5. **Read DVFS counters** back (this run's `dropped_ticks`, `max_period_us`, `max_pass_us`).
6. **Restore** the default interval (100 ms) in `TearDown` and confirm.

At the end, if `TT_TELEM_JSON` is set, every accumulated sample for the chip is written to JSON.

## Pass / fail

This is a gathering harness, so pass/fail only covers what it controls: the interval was **set**,
**applied** (live read-back equals the knob), and **restored** to 100 ms. The gathered telemetry, the
DVFS counters, and the publish-rate stats are **logged, not asserted**.

## Build & run

```bash
cd ~/tt-umd
cmake --preset tests            # once
cmake --build build --target umd_microbenchmark
./build/test/umd/microbenchmark/umd_microbenchmark --gtest_filter='BlackholeTelemetryInterval.*'
```

## Environment arguments


| Env var                | Default                                                            | Meaning                                                                                                                                                                                                         |
| ---------------------- | ------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `TT_TELEM_INTERVAL_MS` | `3`                                                                | Interval knob set on the firmware for the run (ms).                                                                                                                                                             |
| `TT_SAMPLE_DURATION_S` | `2.0`                                                              | Sampling window per device (seconds).                                                                                                                                                                           |
| `TT_TELEM_POLL_US`     | `interval/3` (e.g. ~1000 at 3 ms)                                  | Heartbeat poll period (µs). `0` = poll flat-out. Keep well below the interval so every update is caught.                                                                                                        |
| `TT_TELEM_TAGS`        | `TIMER_HEARTBEAT,VCORE,TDP,TDC,AICLK,ASIC_TEMPERATURE,INPUT_POWER` | Columns to accumulate/emit: comma list of tag names or numeric ids, or `all`. The full table is still read every sample; this only selects which columns are kept. Unknown/absent tags are skipped with a note. |
| `TT_TELEM_JSON`        | *(unset)*                                                          | If set, path to write the accumulated samples as JSON at the end.                                                                                                                                               |
| `TT_TELEM_CHIP`        | `0`                                                                | Index (into the enumerated device list) of the single chip to run on. Only this chip is opened/touched.                                                                                                         |


### Example

```bash
TT_SAMPLE_DURATION_S=5 \
TT_TELEM_TAGS=VCORE,AICLK,ASIC_TEMPERATURE \
TT_TELEM_JSON=/tmp/telem.json \
./build/test/umd/microbenchmark/umd_microbenchmark --gtest_filter='BlackholeTelemetryInterval.*'
```

## Output

### Console (per chip)

```
[  TELEM   ] chip 0: duration=5s, poll=1000us
[  TELEM   ]   host sweep rate:   ... Hz (... polls)
[  TELEM   ]   FW publish rate:   333 Hz (~3 ms), ... samples
[  TELEM   ]   DVFS counters (this run): dropped_ticks=0 max_period_us=... max_pass_us=...
```

- **host sweep rate** — how fast the host polled the heartbeat (measurement ceiling, not data rate).
- **FW publish rate** — how often the firmware published a new sample (should match the interval).
- **DVFS counters** — this run's dropped ticks, longest start-to-start pass spacing, longest pass.

### JSON (`TT_TELEM_JSON`)

```json
{
  "interval_ms": 3,
  "duration_s": 5,
  "chips": [
    {
      "chip": 0,
      "poll_us": 1000,
      "host_sweep_hz": 0,
      "fw_publish_hz": 0,
      "fw_updates": 0,
      "dvfs_counters": {"dropped_ticks": 0, "max_period_us": 0, "max_pass_us": 0},
      "tags": ["TIMER_HEARTBEAT", "VCORE", "AICLK", "ASIC_TEMPERATURE"],
      "samples": [
        {"host_time_ns": 1764072000123456789, "values": [10231, 713, 800, 3248786]},
        {"host_time_ns": 1764072000126512345, "values": [10232, 713, 800, 3265586]}
      ]
    }
  ]
}
```

Each `samples` entry is an object with an explicit `host_time_ns` — the host wall-clock time (ns since
the Unix epoch) at which the host detected that FW sample — and a `values` array, one value per tag in
`"tags"` order. Include `TIMER_HEARTBEAT` in `TT_TELEM_TAGS` if you want the per-sample heartbeat
counter in the rows.

**Values are raw u32, not engineering units** — the firmware packs most tags in a fixed-point or
bit-packed form. For decoding, please refer to
https://docs.tenstorrent.com/tt-system-firmware/doxygen//group__telemetry__tags.html.

## Host read overhead

Measured on a single p150a (table = 86 entries / 344 B):

- **Full-table read (`read_all_entries`): ~17 µs** (p50 ≈ 17.0, p90 ≈ 17.7, rare max ≈ 66 µs). The 344 B
  copy is sub-µs; almost all of this is the per-read **TLB reconfigure** (BAR0 register writes in UMD),
  not data movement. This is the per-chip overhead per sample.
- **Single-word heartbeat poll: ~2.6 µs** — much cheaper, since it hits a TLB-cached window.

## Using it as a skeleton to run a workload during the window

The run is a linear sequence; the measurement window is the `sample_run(...)` call. All stages prior
and after this are the setup and tear down required to use this as a skeleton/template.