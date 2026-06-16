# IDEA-008: Eliminate GC pause jitter from the control loop

## Background

MicroPython uses a mark-and-sweep garbage collector that stops all execution while it
runs. At 300 Hz the inner loop period is 3.33 ms; a GC pause of even 2-5 ms causes a
missed cycle, stretching `dt` for that iteration.

The primary allocator in the hot path is `TelemetryRecorder.record()` in
`src/telemetry/recorder.py:164` -- a `"{...}".format(...)` call that creates a new
~180-char string every `sample_every` cycles, followed by `(line + "\n").encode()` in
`SdSink.write()` adding two more temporaries. At `sample_every=20` and 300 Hz that is
15 allocations/sec. Each string is short-lived and freed, but MicroPython's GC has no
generational optimisation -- every collection walks the full heap.

### Empirical findings (2026-05-31 runs)

Measured from actual flight logs (`2026-05-31_21-*`):

- **se=20 production runs**: effective rate 250--267 Hz (3.75--4ms/cycle), not 300 Hz.
  Systematic +13% overhead per cycle; p99 row gap 93--125ms, max 106--149ms.
- **se=1 full-resolution run**: effective rate 143 Hz. Cause: `format()` takes ~3ms per
  call; every cycle pays it, doubling loop time. No SD sector-write periodicity (mod-3
  check was flat), confirming the bottleneck is string formatting, not SPI flushes.
- **GC pauses in se=1 run**: 43 gaps > 15ms out of 4303 rows (0.6%), max 31ms.
  These are genuine GC pauses; they do not appear at the mod-3 SD-write cadence.

### The D-term spike hypothesis is NOT confirmed

The original intuition -- GC pause -> large `dt` -> amplified D-term -> overshoot --
does not hold. At gap rows > 15ms:

  RATE_D mean: 0.014 (spike rows) vs 0.044 (baseline) -- ratio 0.3x
  ANG_D mean:  0.07  (spike rows) vs 2.53  (baseline) -- ratio 0.03x

D-terms are 3--30x SMALLER at pause rows. Large `dt` in the denominator
(`kd * delta_error / dt`) wins: the lever barely moves in 20ms while motors hold their
last output, so `delta_error` is small AND `dt` is large. The net effect is suppression,
not amplification.

GC pauses are still undesirable (missed sensor cycles, unpredictable loop timing), but
they are not the direct cause of D-term transients or overshoots.

## Diagnostic (no code change required)

`T_MS` inter-row gaps reveal window-level timing anomalies. At `sample_every=20` and the
observed 267 Hz effective rate, the expected gap is ~75ms. Use actual p50 as baseline,
not the theoretical value:

```python
import pandas as pd
df = pd.read_csv("log.csv")
gaps = df["T_MS"].diff()
baseline_ms = gaps.median()
spikes = df[gaps > baseline_ms * 1.5]
print(spikes[["T_MS", "RATE_D", "ANG_D"]])
```

**Limitation:** the inter-row gap covers the entire `sample_every` window. A GC pause
in cycle 3 of 20 will stretch the gap, but D-terms at the logged row (cycle 20) may
already have recovered. The gap is useful for counting events; it cannot pinpoint
which cycle in the window was affected or correlate with the PID state at that moment.
See Option D below for per-cycle visibility.

## Option A: Deterministic GC (low complexity, recommended first step)

Disable auto-GC and collect manually at a known-safe moment in the outer-loop block --
after `update_sensors()` returns new data and before the angle PID runs:

```python
import gc
gc.disable()   # once, before the control loop

# inside the while loop, at the top of the outer-loop block:
if outer_counter >= outer_ticks:
    outer_counter = 0
    gc.collect()           # deterministic pause here
    iqr, iqi, iqj, iqk = imu.game_quaternion
    ...
```

The pause still exists but lands at a fixed phase: after sensor data is consumed and
before any PID output is computed. No mid-loop surprises. The rate loop is unaffected --
the angle PID output becomes the new rate setpoint immediately after collect() returns.

**Safe moment rationale:** outer_counter just rolled, sensor data just consumed, motor
outputs not yet computed. GRV filter lag (~10 ms) already dominates the outer loop's
phase error; the GC pause (~2-5 ms) is inside that budget.

**Cost:** the GC pause does not go away, it just moves to a predictable moment. If
memory pressure is high enough that a single outer-cycle collection is insufficient,
heap will exhaust. Monitor with `gc.mem_free()` printed at run end.

## Option B: Eliminate hot-path allocations (medium complexity)

Remove the string allocation from the record() hot path entirely. Pre-allocate a
`bytearray` scratch buffer and format floats directly into it:

- Replace `"{...}".format(...)` with field-by-field writes into a fixed bytearray.
- Replace `(line + "\n").encode()` by writing the bytearray directly to `SdSink`.
- Use `memoryview` slices to avoid copying.

MicroPython does not have `struct.pack_into` for arbitrary-precision floats, so this
requires a small manual formatter or a helper that writes ASCII decimal into a bytearray
in-place. Non-trivial to get right but eliminates the allocation entirely.

**Benefit:** with no allocation in the hot path, GC runs are rare enough that auto-GC
timing becomes irrelevant. Option A becomes unnecessary.

**Drawback:** more implementation work, harder to read than format().

## Option C: Both (highest robustness)

Combine A and B: eliminate hot-path allocations (Option B) and keep `gc.disable()` with
a periodic `gc.collect()` at the outer-loop safe point (Option A). Option B reduces how
often collect() does useful work; Option A ensures any residual allocation from BNO085
driver internals or one-off paths never fires at a random moment.

## Option D: Record per-cycle dt in telemetry (low complexity, high diagnostic value)

Add two columns to the CSV so every logged row carries direct timing evidence:

- **`DT_MS`** -- dt of the cycle being logged. This is the same `dt_ms` the PID used for
  that row. Correlates directly with RATE_D and ANG_D in the same row without any
  post-hoc diffing.
- **`MAX_DT_MS`** -- maximum dt seen across the entire `sample_every` window since the
  last logged row. Catches GC pauses in any of the 20 cycles, including the 19 that are
  silently dropped. A large `MAX_DT_MS` with a normal `DT_MS` means the pause landed in
  a non-logged cycle -- currently invisible.

**Why `T_MS` diff alone is insufficient:**
- `T_MS` diff gives window duration but not which cycle was slow.
- A 20ms GC pause in cycle 3 of 20 stretches the row gap by 20ms. At the logged row
  (cycle 20), `DT_MS` is normal and PID state has recovered -- the gap looks alarming
  but correlates with nothing. `MAX_DT_MS` flags the same event at the right row.

**Implementation sketch:**

In `TelemetryRecorder`:
```python
def __init__(self, sample_every, sink):
    ...
    self._max_dt_ms = 0

def record(self, t_ms, dt_ms, ...):   # dt_ms added
    if dt_ms > self._max_dt_ms:
        self._max_dt_ms = dt_ms
    self._counter += 1
    if self._counter < self._sample_every:
        return
    self._counter = 0
    max_dt = self._max_dt_ms
    self._max_dt_ms = 0            # reset window
    line = "{},{},{},...".format(t_ms, dt_ms, max_dt, ...)
    self._sink.write(line)
```

In `flight.py`, pass `dt_ms` to `telemetry.record()` -- it is already computed as
`dt_ms = utime.ticks_diff(now_ms, prev_ms)` at line 143.

**CSV impact:** two new columns (`DT_MS`, `MAX_DT_MS`) appended at the end of each row.
All analysis scripts use `csv.DictReader` (column-name lookups) -- they silently ignore
unknown columns. No analysis-script changes required.

**What this enables:**
- Scatter plot `MAX_DT_MS` vs overshoot magnitude across runs -- direct GC/SD causal test.
- Flag any row where `MAX_DT_MS > 2 * DT_MS` (pause in a non-logged cycle).
- After applying Option A or B, confirm `MAX_DT_MS` distribution tightens.

## Gradient summary

| Option | Complexity | Eliminates pause? | Eliminates jitter? | Diagnostic value | Status |
|--------|------------|-------------------|--------------------|-----------------|--------|
| A: Manual GC | Low | No (moves it) | Yes -- pause is now deterministic | None | Done 2026-06-05 (time-gated, no gc.disable) |
| B: No hot-path alloc | Medium | Mostly | Yes -- GC rarely triggered | None | Done 2026-06-03 (binary struct encoding) |
| C: A + B | Medium | Mostly | Yes -- belt and suspenders | None | Done (A + B both implemented) |
| D: DT_MS + MAX_DT_MS columns | Low | No | No | High -- confirms/refutes causal chain | Done 2026-06-03 |

**Implemented (2026-06-03 to 2026-06-05):**
- **Option D** done first: `DT_MS`/`MAX_DT_MS` columns in binary record. Confirmed GC
  causes MAX_DT spikes; D-term suppression hypothesis verified (D-terms are smaller,
  not larger, at spike rows).
- **Option B** done: binary `struct.pack_into` replaces CSV formatting; zero heap
  allocation per record.
- **Option A** done with modification: `gc.disable()` was attempted but failed
  immediately -- Core 1's DShot loop (`motor_throttle_group.py`) allocates at 1 kHz;
  without auto-GC it exhausts the heap before the first manual collect fires
  (`MemoryError` in `sendThrottleCommand`). Fix: time-gated collect -- `gc.collect()`
  fires at the outer-tick boundary when >= 800ms have elapsed since the last collection.
  Auto-GC remains active as a safety net for Core 1; the timed collect handles the
  common case deterministically. MAX_DT reduced from 62ms to 19ms.

## Stage 2: Separate GC telemetry from algorithm telemetry

**Problem with Stage 1:** `DT_MS` and `MAX_DT_MS` in `log.csv` conflate two concerns:
- `log.csv` is algorithm telemetry -- it describes flight behaviour, feeds the analyser
  pipeline (gate, verdict, diagnose, plots), and archives the PID state history.
- Cycle timing is performance telemetry -- it describes the runtime behaviour of the
  control loop implementation, not the physics of the flight.

Mixing them makes `log.csv` harder to read, complicates the analyser pipeline if timing
columns are later exposed to it, and couples two independently evolving schemas.

**Stage 2 options:**

*Option E: Separate CSV file per session (`timing.csv`)*
Write a parallel `YYYY-MM-DD_hh-mm-ss/timing.csv` alongside `log.csv` at the same
cadence. Columns: `T_MS,DT_MS,MAX_DT_MS` (possibly add `GC_FREE` = `gc.mem_free()`).
`SdSink` grows a second file handle; `TelemetryRecorder` routes to the correct sink.
Desktop: analyser ignores `timing.csv` unless explicitly called; a separate
`gc_analysis.py` script reads it.

*Option F: Stdout stream (USB CDC / REPL)*
Print GC events to stdout gated on a threshold (`if max_dt > THRESHOLD_MS: print(...)`).
Read via `mpremote` or a serial terminal during the run. Zero SD overhead; no post-hoc
file. Limitation: `print()` blocks if the USB buffer fills (host not reading fast enough)
-- dangerous in the control loop for any high-rate path. Safe only for rare events
(< ~10/sec). USB CDC bandwidth (~100 KB/s) exceeds the SD effective rate (~100 KB/s),
but per-call Python overhead makes it unsuitable for every-cycle logging.

*Option G: Binary sidecar*
Pack `(t_ms: u32, dt_ms: u16, max_dt_ms: u16)` = 8 bytes per row into a binary file.
Quarter the write volume vs CSV. Requires a PC-side decoder.

**Recommended Stage 2 path:** Option E -- parallel `timing.csv` with its own sink.
Remove `DT_MS`/`MAX_DT_MS` from `log.csv` once Stage 2 is implemented. Stage 1
columns are explicitly a diagnostic scaffold, not a permanent schema addition.