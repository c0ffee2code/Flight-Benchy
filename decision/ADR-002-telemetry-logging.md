# ADR-002: Telemetry Logging with Adalogger PiCowbell

**Status:** Draft
**Date:** 2026-02-08
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
| **I2C** | STEMMA QT connector on GPIO 4 (SDA) / GPIO 5 (SCL) — second I2C bus |
| **SD detect** | Optional, GPIO 15 |

### Pin Conflict

The Adalogger's I2C bus (GPIO 4/5) and optional SD card detect (GPIO 15) conflict with current pin assignments:

| GPIO | Current use | Adalogger use |
|------|-------------|---------------|
| 4 | Motor 1 DShot | I2C SDA (RTC + STEMMA QT) |
| 5 | Motor 2 DShot | I2C SCL (RTC + STEMMA QT) |
| 15 | Button Y | SD card detect (optional) |

## Decision

### Reassign motor pins

Move DShot motor outputs to unused GPIOs. The PIO state machine can use any GPIO pin — the assignment is purely software. Candidate pins:

| Signal | New GPIO | Rationale |
|--------|----------|-----------|
| Motor 1 DShot | GPIO 6 | Adjacent to current pins, no conflicts |
| Motor 2 DShot | GPIO 7 | Adjacent to current pins, no conflicts |

This frees GPIO 4/5 for the Adalogger's I2C bus (RTC + STEMMA QT) and preserves the existing I2C bus on GPIO 0/1 for sensors (AS5600 + BNO085).

SD card detect on GPIO 15 — skip it. The SD library can detect the card by attempting to mount. Button Y (GPIO 15) is more valuable for the UI.

### Resulting I2C topology

Two independent I2C buses:

```
I2C Bus 0 (GPIO 0/1, 400 kHz)     I2C Bus 1 (GPIO 4/5, 100 kHz)
├── AS5600 encoder  [0x36]          ├── PCF8523 RTC  [0x68]
└── BNO085 IMU      [0x4A]          └── STEMMA QT (future sensors)
```

Separating the RTC onto its own bus avoids adding traffic to the sensor bus, which is latency-sensitive for the control loop.

### Log format

CSV with pipe delimiter for grep-friendliness (consistent with AS5600 diagnostic format):

```
T_RTC,T_MS,ENC_DEG,IMU_DEG,ERR,P,I,D,M1,M2
2026-02-08T14:30:01,12345,+0.5,+0.8,-0.3,2.5,0.1,0.0,165,135
```

Fields:
- `T_RTC` — Wall clock from PCF8523 (ISO 8601, second resolution)
- `T_MS` — `ticks_ms` since boot (sub-ms precision for inter-sample timing)
- `ENC_DEG` — AS5600 angle in degrees (ground truth)
- `IMU_DEG` — BNO085 angle in degrees (when available, empty otherwise)
- `ERR` — PID error term
- `P`, `I`, `D` — Individual PID contributions
- `M1`, `M2` — Motor throttle values

### Write strategy

SD card writes are slow and unpredictable (flash erase cycles). To avoid stalling the PID loop:

- **Buffer in RAM** — accumulate N lines (e.g., 50 = 1 second at 50 Hz) in a pre-allocated list
- **Flush periodically** — write buffer to SD during a non-critical window (e.g., after display update)
- **Flush on disarm** — write remaining buffer when transitioning to DISARMED state
- **File naming** — `LOG_NNNN.csv` with incrementing counter, one file per experiment run

### File organization

```
SD card root/
├── LOG_0001.csv
├── LOG_0002.csv
└── ...
```

Each file gets a header row on creation. RTC timestamp in the first data row provides the wall clock reference; subsequent rows use `T_MS` deltas for precise inter-sample timing.

## Consequences

### Positive

- Real-time timestamps enable correlating experiments across days/sessions
- SD card provides practically unlimited storage for extended experiments
- Separate I2C bus for RTC avoids impacting sensor latency
- CSV format works directly with pandas, matplotlib, and the existing `analyse_report_rate.py` from BNO085 repo
- RAM buffering protects PID loop from SD write latency

### Risks

- **SPI bandwidth** — SD writes share SPI pins with nothing else currently, but SPI transactions take time. Buffering mitigates this.
- **Power loss** — Unflushed buffer is lost on crash/power loss. Acceptable for a test bench; could add periodic flush at cost of occasional PID jitter.
- **SD card reliability** — Frequent small writes can wear SD cards. Log files are small (few MB per run), so this is not a practical concern.
- **PCF8523 drift** — RTC accuracy is ±2 ppm (~1 minute/year). Adequate for experiment correlation; not a precision time source.

## Dependencies

- PCF8523 MicroPython driver (Adafruit provides CircuitPython; may need port or lightweight custom driver)
- `sdcard.py` MicroPython driver (available in micropython-lib)
- Motor pin reassignment in `main.py` and CLAUDE.md (GPIO 4/5 → GPIO 6/7)
