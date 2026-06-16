# IDEA-007: Reduce SD write latency impact on control loop

## Background

`SdSink.write()` buffers rows into a 512-byte sector buffer (`_SECTOR = 512` in
`recorder.py`). Every ~3 rows the buffer fills and a sector write fires (~5ms, measured
2026-05-31). `sample_every=3` destabilised the rate PID (block writes every ~34ms).
Safe range currently `sample_every=20-60`.

Two cost components compound: (1) the ~5ms SD internal flash commit -- physics,
unavoidable; (2) FAT metadata writes (cluster chain extension, directory entry update) --
happen every sector write as the file grows. Option A targets (2). Options B and C
target both.

**Option A is implemented** (merged 2026-05-31): `SdSink` pre-allocates the log file
to `telemetry.preallocate_bytes` bytes (default 1 MB) at session start, eliminating
cluster chain extension mid-run. SPI clock raised to 25 MHz. Component (2) is resolved;
component (1) -- the raw 5ms flash commit per 512-byte sector -- remains.

The SD card physical sector size is **fixed at 512 bytes** by the SD specification.
It cannot be changed in software. However, the SdSink write buffer size and the SPI
multi-block write command are both software-controllable levers (see Option D).

## Option A: Pre-allocate the log file (low complexity)

At `SdSink.init_session`, create the log file and pre-extend it to a fixed size (e.g.
8 MB, ~70,000 rows) with a fill byte. This allocates the entire cluster chain upfront.

During the run: seek to cursor and overwrite sequentially. Cluster chain already exists --
no allocation stalls mid-run, only sector data updates. Directory entry (file size) still
needs flushing on close.

At session end: truncate or rewrite to actual byte count so the CSV contains no garbage.

**Benefit:** removes cluster allocation overhead from the hot path. Baseline 5ms block
write remains. More consistent latency -- the control loop cares about consistency, not
absolute speed.

**Drawbacks:** reserves ~8 MB at session start; MicroPython `f.seek + f.write` API needs
verification for pre-extend behaviour. Power cut leaves a full-size file padded with
garbage (recoverable, same as now).

**New config key:** `telemetry.preallocate_bytes` (default 8 MB, 0 = disable).

## Option B: Write raw SD sectors, skip FAT (medium complexity)

Skip FAT entirely during recording. Pick a reserved block range at the end of the card
(partition table awareness needed), write rows as raw 512-byte blocks via
`sdcard.writeblocks()`. PC-side tool converts raw dump to CSV after the run.

**Benefit:** eliminates all FAT metadata overhead. 5ms block write remains (SD physics),
but that is now the only cost. No directory entry, no cluster chain.

**Drawbacks:** requires implementing a minimal block allocator and a PC-side converter
script. Recovery on power cut is harder -- no file system to mount, must know the block
range. Moves complexity to the PC rather than the Pico.

## Option C: Dedicated logger RP2350 (high complexity, highest isolation)

Add a second Pico 2 (~$5) wired to the main Pico via SPI. Main Pico DMA-blasts telemetry
packets at each inner loop tick (microseconds of overhead). Logger Pico owns FAT, SD card,
buffering entirely.

```
Main RP2350  -- SPI -->  Logger RP2350  -->  SD card
(control loop)           (FAT / buffering)
```

**Benefit:** the control loop is completely immune to SD write timing. Logger Pico also
gains two extra cores, 520 KB RAM, extra DMA channels, extra PIOs -- buffering, optional
compression, or binary packing become trivial.

**Drawbacks:** extra hardware, wiring, inter-MCU protocol design (framing, overflow
handling, session start/end signalling). Highest one-time engineering cost, but makes
logging a non-issue permanently.

## Option D: Larger write buffer + CMD25 multi-block write (low complexity)

The SD SPI protocol has two write commands:
- **CMD24** (WRITE_BLOCK): writes one 512-byte sector. Cost: ~5ms per call.
- **CMD25** (WRITE_MULTIPLE_BLOCK): writes N sectors in one command. The SD card
  pipelines flash programming across sectors. Measured savings: ~6-8ms for 4 sectors
  vs 4 × CMD24 = ~20ms. The driver already implements CMD25 (`sdcard.py:289-298`) --
  it is selected automatically when `writeblocks(block_num, buf)` receives a buffer
  larger than 512 bytes (`nblocks > 1`).

**The lever:** increase `_SECTOR` in `recorder.py` from 512 to 2048 (or any multiple of
512). The buffer then holds ~11 rows before flushing (vs ~3 rows now). Two effects:

1. **Flush frequency drops 4×** -- independently useful regardless of CMD25. At
   `sample_every=20`, flushes go from every ~225ms to every ~825ms. At `sample_every=5`,
   from every ~56ms to every ~206ms.

2. **CMD25 activated** -- IF FatFs (`self._f.write()`) passes the full 2048-byte buffer
   to `writeblocks()` as a single call, the driver uses CMD25 and the 4-sector write
   costs ~6-8ms instead of 4 × 5ms = 20ms.

**Uncertainty:** FatFs may split a 2048-byte `f.write()` internally into separate
512-byte block writes, reverting to CMD24. This cannot be confirmed without measurement.
The flush-frequency reduction is guaranteed regardless.

**To guarantee CMD25:** call `sdcard.writeblocks()` directly, bypassing FatFs (overlaps
with Option B). This is the reliable path but requires restructuring `SdSink` to manage
block addresses manually on the pre-allocated region.

**Implementation:** change `_SECTOR = 512` to `_SECTOR = 2048` in `recorder.py` and
change `_write_buf = bytearray(_SECTOR)` accordingly. No other changes needed.
Verify `MAX_DT_MS` distribution tightens in subsequent runs (use IDEA-008 Option D
columns to measure).

## Option E: Exact-size log file — log.tmp → log.bin session pattern (implemented 2026-06-05)

Pre-allocation fills the log file with null bytes. Pico-side `f.truncate()` was
attempted to strip the padding at close but raises `AttributeError` — MicroPython's
`FileIO` does not implement `truncate()`.

**Implemented solution:** write to `log.tmp` during the session; at `close()`, copy
exactly `actual_bytes` from `log.tmp` to `log.bin` in 512-byte chunks, then remove
`log.tmp`. The SD card always shows one of two states:
- `log.tmp` present, `log.bin` absent — session in progress
- `log.bin` present at exact size, `log.tmp` absent — session complete

`pull_flights.py` decodes `log.bin` (binary struct → CSV string) and writes `log.csv`
to the local `test_runs/flights/` folder. Transfer is proportional to actual data:
a 60s run at `se=20` transfers ~79 KB instead of 1 MB.

**No null-byte concern in the analysis pipeline:** `pull_flights.py` decodes the binary
format record-by-record (`len(raw) // 88`), producing a clean CSV. The null padding
never reaches the PC-side loader.

## Gradient summary

| Option | Complexity | Reduces spike frequency? | Reduces spike duration? | Status |
|--------|------------|--------------------------|-------------------------|--------|
| A: Pre-allocate | Low | No | No (removes FAT overhead only) | Done 2026-05-31 |
| B: Raw sectors | Medium | Yes | Yes (CMD25 guaranteed) | Backlog |
| C: Second MCU | High | Yes (fully isolated) | Yes (fully isolated) | Backlog |
| D: Larger buffer | Low | Yes (4x fewer flushes) | Maybe (CMD25 if FatFs cooperates) | Backlog |
| E: Exact-size log (log.tmp->log.bin) | Low | N/A | N/A | Done 2026-06-05 |

Option D is the next obvious step -- a one-line change (`_SECTOR = 2048`). Measure
with `MAX_DT_MS` before and after. If CMD25 benefit is confirmed, B becomes less
urgent. If not, B remains the reliable path to eliminate multi-sector write overhead.