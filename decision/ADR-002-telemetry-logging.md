# ADR-002: Telemetry Logging with Adalogger PiCowbell

**Status:** Accepted
**Date:** 2026-02-08
**Updated:** 2026-02-14
**Context:** Black box logging for PID tuning, BNO085 driver development, and post-experiment analysis

## Context

The test bench needs persistent telemetry logging for two use cases:

1. **PID tuning and control loop analysis** — Recording angle, PID terms (P/I/D contributions), and motor throttles over time to diagnose oscillation, overshoot, and steady-state error. Without logs, tuning is guesswork.

2. **BNO085 driver development** — The IMU driver needs characterization before it can replace the AS5600 encoder as the primary control input. Logging both sensors simultaneously with real-time timestamps enables measuring IMU lag, angle error, drift, and report rate consistency.

Currently there is no persistent logging — the only output is the display and serial print. Data is lost when the experiment ends.

### Requirements

- Real-time timestamps (wall clock, not just `ticks_ms`) for correlating logs across experiments
- Persistent storage that survives power cycles and holds multiple experiment runs
- Minimal impact on the 50 Hz control loop — logging must not starve the PID
- CSV format for easy analysis with Python/pandas on PC

## Hardware Choice

**Adafruit PiCowbell Adalogger for Pico** ([product 5703](https://www.adafruit.com/product/5703))

Stacks directly onto the Raspberry Pi Pico 2. Provides:

| Component | Details |
|-----------|---------|
| **RTC** | PCF8523, CR1220 coin cell backup, I2C address `0x68` |
| **SD card** | MicroSD via SPI — MOSI=GPIO 19, MISO=GPIO 16, SCK=GPIO 18, CS=GPIO 17 |
| **I2C** | STEMMA QT connector on GPIO 4 (SDA) / GPIO 5 (SCL) |
| **SD detect** | Optional, GPIO 15 |

### Pin Conflict

The Adalogger's I2C bus (GPIO 4/5) and optional SD card detect (GPIO 15) conflict with current pin assignments:

| GPIO | Current use | Adalogger use |
|------|-------------|---------------|
| 4 | Motor 1 DShot | I2C SDA (RTC + STEMMA QT) |
| 5 | Motor 2 DShot | I2C SCL (RTC + STEMMA QT) |
| 15 | Button Y | SD card detect (optional) |

### I2C bus mapping gotcha

GPIO 4/5 are **I2C0 alternate pins** on RP2350, not I2C1 as the Adalogger documentation implies. Since I2C0 is already used for sensors (AS5600 + BNO085) on GPIO 0/1, we cannot assign GPIO 4/5 to a second hardware I2C peripheral. The RTC is accessed via **SoftI2C** (bit-banged) instead. This is acceptable because the RTC is only read once per session — there is no ongoing bus traffic.

## Decision

### Reassign motor pins

Move DShot motor outputs to unused GPIOs. The PIO state machine can use any GPIO pin — the assignment is purely software.

| Signal | New GPIO | Rationale |
|--------|----------|-----------|
| Motor 1 DShot | GPIO 6 | Adjacent to current pins, no conflicts |
| Motor 2 DShot | GPIO 7 | Adjacent to current pins, no conflicts |

This frees GPIO 4/5 for the Adalogger's RTC (via SoftI2C) and preserves the existing I2C bus on GPIO 0/1 for sensors (AS5600 + BNO085).

SD card detect on GPIO 15 — skip it. The SD library can detect the card by attempting to mount. Button Y (GPIO 15) is more valuable for the UI.

### Resulting bus topology

```
I2C Bus 0 (GPIO 0/1, 400 kHz)     SoftI2C (GPIO 4/5, 100 kHz)
├── AS5600 encoder  [0x36]          └── PCF8523 RTC  [0x68]
└── BNO085 IMU      [0x4A]

SPI Bus 0 (GPIO 16/17/18/19)
└── MicroSD card
```

Separating the RTC onto a bit-banged bus avoids adding traffic to the sensor bus, which is latency-sensitive for the control loop. SoftI2C is acceptable because the RTC is read once per session for filename generation only.

### Timestamp strategy

The RTC is used **only once per session** to generate a human-readable log filename. All row timestamps use `time.ticks_ms()` (Pico's internal microsecond timer), which is a near-free register read. This avoids the cost of an I2C transaction per log row and provides the precise inter-sample timing needed for PID analysis. The RTC wall-clock time in the filename provides session correlation across days.

### Log format

CSV with comma delimiter (standard CSV):

```
T_MS,ENC_DEG,IMU_DEG,ERR,P,I,D,PID_OUT,M1,M2
12345,+0.5,,+0.5,2.50,0.10,0.00,2.60,303,297
```

Fields:
- `T_MS` — `ticks_ms` since boot
- `ENC_DEG` — AS5600 angle in degrees (ground truth)
- `IMU_DEG` — BNO085 angle in degrees (empty until M2)
- `ERR` — PID error term
- `P`, `I`, `D` — Individual PID contributions
- `PID_OUT` — Raw PID output before clamping to motor range (shows PID saturation)
- `M1`, `M2` — Motor throttle values (integers)

### Write strategy

Started with direct writes (no RAM buffering) for simplicity. Each `record()` call writes one line directly to the open file. This is the simplest approach and avoids buffer management complexity. RAM buffering can be added later if SD write latency causes measurable PID jitter.

### File naming and organization

```
SD card root/
└── blackbox/
    ├── log_2026-02-14_15-30-00.csv
    ├── log_2026-02-14_16-45-12.csv
    └── ...
```

Files use the pattern `log_YYYY-MM-DD_hh-mm-ss.csv` with the RTC timestamp captured at session start. The `blackbox/` subdirectory keeps telemetry logs separate from other data that may be stored on the SD card. Each file gets a CSV header row on creation.

### Sampling & decoupling

The telemetry pipeline separates data collection from I/O through a facade pattern:

- **`TelemetryRecorder`** — facade called from the main loop. Accepts all telemetry fields per cycle, handles decimation, and delegates output to a pluggable sink.
- **`PrintSink`** — prints CSV rows to REPL serial console. No buffering needed.
- **`SdSink`** — owns the full SD card lifecycle (mount, file create, write, unmount). Constructed with SPI and RTC pin numbers; reads the RTC once to generate the filename; mounts the SD card; creates the log file. On close, flushes the file and unmounts the SD card.

The `SdSink` encapsulates all hardware interaction (SPI, SoftI2C, SD card driver, filesystem mount/unmount) so the main loop only deals with pin constants and the sink interface.

Configurable decimation via `TELEMETRY_SAMPLE_EVERY` constant in `main.py`:
- Set high (e.g., 10) for REPL output to avoid serial flood
- Set to 1 for SD card logging to capture full-rate data
- Decimation is cycle-count based: every N-th call to `record()` emits a row

### SD card driver fix

The upstream `sdcard.py` from micropython-lib has a bug: the CRC byte's stop bit (bit 0) is not set for commands after CMD0/CMD8. The SD SPI command frame requires the end bit to always be `1` (see [SD SPI command format](https://elm-chan.org/docs/mmc/mmc_e.html)). Most cards are lenient, but some (e.g., Philips SDHC) strictly enforce it and refuse all commands after CMD8. Fix: `buf[5] = crc | 0x01` in `sdcard.py`.

## Consequences

### Positive

- RTC timestamps in filenames enable correlating experiments across days/sessions
- SD card provides practically unlimited storage for extended experiments
- SoftI2C for RTC avoids impacting sensor I2C bus latency
- CSV format works directly with pandas, matplotlib, and the existing `analyse_report_rate.py` from BNO085 repo
- `SdSink` encapsulates all SD/RTC hardware — main loop stays clean
- Direct writes keep the implementation simple; buffering can be added if needed

### Fail-fast error handling

Telemetry I/O errors (SD write failure, card ejection, filesystem full) are **not caught** inside the control loop. An exception in `SdSink.write()` propagates up, exits the PID loop, and the `finally` block in `main()` stops the motors.

This is intentional: the test bench exists to collect data for analysis and tuning. If telemetry is not being captured, continuing the run produces no useful output — the lever stabilizing without data is wasted effort. Failing fast also ensures the operator notices the problem immediately rather than discovering a truncated log file later.

A production flight controller would isolate telemetry failures from the control loop. That trade-off does not apply here.

### Risks

- **SPI bandwidth** — SD writes share SPI pins with nothing else currently, but SPI transactions take time. If PID jitter is observed, add RAM buffering.
- **Power loss** — With direct writes, the last few lines may be lost. Acceptable for a test bench.
- **SD card reliability** — Frequent small writes can wear SD cards. Log files are small (few MB per run), so this is not a practical concern.
- **PCF8523 drift** — RTC accuracy is ±2 ppm (~1 minute/year). Adequate for filename timestamps; not a precision time source.

## Dependencies

- `sdcard.py` MicroPython driver (from micropython-lib, with stop bit fix applied)
- Motor pin reassignment in `main.py` (GPIO 4/5 → GPIO 6/7) — done