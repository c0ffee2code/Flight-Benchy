# DR-016: Deterministic Telemetry I/O — Binary Encoding, SD Pre-allocation, and GC Control

**Status:** Implemented — 2026-06-05
**Relates to:** DR-002 (telemetry logging), DR-008 (cascaded PID)

## Problem

The CSV telemetry pipeline introduced two sources of irregular delay in the control loop. Both manifested as `MAX_DT_MS` spikes — cycles where elapsed time was 3–18× the nominal 3.33ms inner loop period. The D-term received an inflated `dt` on those cycles and produced overcorrection impulses.

**SD write spikes:** `SdSink.write()` buffered rows into a 512-byte sector. Every ~3 rows the buffer filled and a sector write fired: ~5ms for the physical SD flash commit, plus FAT cluster-chain extension overhead as the log file grew into new clusters. At `sample_every=3` sector writes fired every ~34ms. These pauses occurred unpredictably mid-loop, inflating `dt_ms` on the affected inner cycle.

**GC pauses from string allocation:** CSV row formatting (f-string + concatenation) allocated heap on every telemetry write. MicroPython's mark-and-sweep GC pauses all execution on both cores when the heap fills. With CSV encoding, GC fired roughly every 783ms with pauses up to 32ms. Switching to binary struct encoding eliminated those per-row allocations but left residual allocations from BNO085 SHTP tuple returns. These continued to trigger GC every ~783ms — now with pauses up to 62ms, because less frequent collection left more live objects for the sweep to scan.

A 62ms pause at 278 Hz = ~18 missed inner cycles. The next cycle's `dt` carried all of that elapsed time into the D-term computation.

## Decision

Three changes applied together to make control loop timing smooth, deterministic, and predictable.

### 1. Binary struct encoding (zero-allocation hot path)

Replace CSV row formatting with `struct.pack_into` into a pre-allocated 88-byte `bytearray`. Record format: `"<I8f11fHHHH"` — one `uint32` (`T_MS`), nineteen `float32` fields, four `uint16` fields (`M1`, `M2`, `DT_MS`, `MAX_DT_MS`). The pack buffer is allocated once at `TelemetryRecorder.__init__` and reused every record. Zero heap allocation per record. Decoding (binary → CSV) is moved to the PC side in `pull_flights.py` via `_decode_log_bin()`.

### 2. SD pre-allocation + log.tmp session pattern

At session start, pre-allocate the log file to `telemetry.preallocate_bytes` bytes (default 1 MB) in one sequential write pass. This reserves the entire FAT cluster chain upfront, eliminating cluster-extension overhead mid-run. SPI clock raised to 25 MHz (from 400 kHz initialisation rate) for the data phase.

During the flight, all writes go to `log.tmp` in the run directory. At session close, `SdSink._finalize_log()` copies exactly `actual_bytes` from `log.tmp` to `log.bin` in 512-byte chunks and removes `log.tmp`. The SD card therefore always shows one of two states: `log.tmp` present and `log.bin` absent (session in progress), or `log.bin` present at correct size and `log.tmp` absent (session complete). Null-padded partial files never appear.

### 3. Time-gated manual GC

At the end of each outer-tick cycle — after both PIDs have run, after motor commands are committed, after encoder is read, after telemetry is written — fire `gc.collect()` if at least 800ms have elapsed since the last collection:

```python
if is_outer_tick and utime.ticks_diff(now_ms, last_gc_ms) >= 800:
    gc.collect()
    last_gc_ms = now_ms
```

The 800ms gate matches the natural auto-GC cadence (observed ~783ms), so collection frequency is unchanged — only the timing is made deterministic. The pause always lands at the same position in the cycle: after the current cycle's output is fully committed. Any delay to the next `update_sensors()` call is absorbed as buffered sensor data; no IMU packet is lost.

## Alternatives Considered

**`gc.disable()` before the control loop:** Initially implemented alongside the time-gated collect. Failed immediately: `gc.disable()` is global to the MicroPython interpreter — Core 1's DShot command loop (in `motor_throttle_group.py`) allocates memory at 1 kHz, and without auto-GC the heap exhausted before the 800ms manual collect fired, producing `MemoryError` in `sendThrottleCommand`. Removed. Auto-GC remains active as a safety net for Core 1; the manual collect handles the common case deterministically.

**`gc.collect()` at every outer tick (100 Hz):** Tested — effective inner loop rate dropped from 278 Hz to 117 Hz. Each `gc.collect()` costs ~16ms regardless of how much garbage exists (full mark-and-sweep of the live heap). At `outer_ticks=3`: average cycle = (3.33 + 3.33 + 3.33 + 16) / 3 = 8.7ms → 115 Hz. Matched observed 117 Hz exactly. Not viable.

**Pico-side `f.truncate()`:** Attempted to call `f.truncate(actual_bytes)` on the open log file to strip null pre-allocation padding at close. MicroPython's `FileIO` does not implement `truncate()` — raises `AttributeError`. Replaced by the `log.tmp → log.bin` copy pattern, which achieves the same result and additionally keeps `log.bin` absent during the session.

**Larger write buffer / CMD25 multi-block write (IDEA-007 Option D):** Increasing `_SECTOR` from 512 to 2048 bytes reduces flush frequency 4× and may activate the SD driver's CMD25 path. Deferred. Pre-allocation eliminates FAT overhead (the dominant source of write latency variability) and GC control caps the pause duration. Option D is an incremental further improvement; the remaining raw 5ms flash-commit cost is constant and therefore does not contribute to D-term spikes.

## Consequences

### Positive

- `MAX_DT_MS` max reduced from 62ms to 19ms; p99 from 43ms to 16ms
- GC spikes occur at a fixed ~800ms interval at a known position in the cycle — predictable and visible in telemetry rather than random
- `log.bin` on the SD card is always absent (in-progress) or correct-sized (complete); no null-padded partial files
- Pull transfer time proportional to actual logged data — a 60s run at `se=20` transfers ~79 KB instead of 1 MB; transfer time drops from ~10s to under 1s
- `sample_every=3` destabilised the rate PID (sector writes every ~34ms); binary encoding + pre-allocation make lower values viable; confirmed safe at `se=5`
- Flight KPIs unchanged: HoldMAE 2.82° on first motor-on GC-controlled run, within the confirmed baseline range of 1.9–3.3°

### Negative / Trade-offs

- Effective inner loop rate ~261–272 Hz vs nominal 300 Hz — residual ~16ms GC pause every 800ms (~2% overhead) plus SD flush latency (~5ms every 5–6 rows) account for the shortfall
- `log.bin` is absent on the SD card during a session; a mid-session pull would find only `log.tmp`. Not a practical concern — pulling during a live session is unsupported
- Binary decode is a required PC-side step: `pull_flights.py:_decode_log_bin()` and `recorder.py:_RECORD_FMT` must stay in sync; a telemetry format change requires updating both

## Verification

| Claim | Evidence |
|---|---|
| `MAX_DT_MS` max < 25ms | 19ms max on run `2026-06-03_09-11-18` |
| GC spike interval ~800ms | 803ms median spike interval on `2026-06-03_09-11-18` |
| `log-truncated` gate passes | PASS on run `2026-06-03_20-53-23` (first run with log.tmp pattern) |
| Flight KPIs unchanged | HoldMAE 2.82°, overshoot 18%, T->SP 3.3s on `2026-06-03_09-11-18` |