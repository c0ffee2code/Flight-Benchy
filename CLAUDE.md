# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. The goal is to experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

**Mechanical Setup:** Carbon fiber drone frame (10 g) + PETG subframe (8 g) forming a swinging lever. Motor shaft separation 6.5 cm. 4 motor mounts, 2 installed (one each end), 2 more planned. Previous frame: 125 g aluminum + PLA, 18.5 cm arm (retired 2026-04-03). Two drone motors with ESCs provide differential thrust for attitude control. A power distribution board supplies the motors.

**Sensor Strategy:**
- **AS5600 magnetic encoder** at the rotation center serves as ground truth reference due to its high precision (~0.09°) and near-instant position output
- **BNO085 IMU** provides the sensor input that a real flight controller would use. Two concurrent reports are enabled: **GRV** (game rotation vector, 0x08, 50 Hz) for the outer angle loop — drift-free via gyro+accel fusion; **calibrated gyroscope** (0x02, 200 Hz) for the inner rate loop — bias-compensated, ~1–2ms latency. See DR-010.

## Development Approach

**Completed:**
- M1 — Single-axis PI(D) controller with AS5600 encoder, validated on hardware. Lever holds at 0° within ±3°. See `decision/DR-001-pid-lever-stabilization.md`.
- M2 — BNO085 IMU as primary PID input (game rotation vector). AS5600 encoder is telemetry-only ground truth. Quaternion telemetry format. BNO085 calibrated (accel/gyro/mag) and tared — DCD and tare persisted to flash. MAE dropped from 22 deg to 2.87 deg. See `decision/DR-005-bno085-pid-input.md`.
- M2a — Black box telemetry logging to SD card via SD card + PCF8523 RTC breakout boards. RTC-timestamped filenames, `ticks_ms` row timing, CSV format. See `decision/DR-002-telemetry-logging.md`.
- M3 — Mixer extraction (`LeverMixer` in `mixer.py`) + telemetry reorganization into `telemetry/` package. Authored source consolidated under `src/`.
- DR-004 — Operator interface: LCD disconnected (resolves SPI0 conflict), buttons + RGB LED only. Motors moved to GPIO 10/11, RGB LED on GPIO 6/7/8. Standard MicroPython firmware.
- M4 — Cascaded PID (angle + rate loops). GRV (50 Hz) for outer loop, calibrated gyro (200 Hz) for inner loop. Baseline 2026-02-22: **0.36° MAE** (3× improvement over 1.12°) after BNO085 re-calibration, correct tare, AXIS_CENTER correction (422→411→406 via precision 3D-printed jig). Post-baseline: lever mechanically balanced, `angle_pid ki=0.05 iterm_limit=5` added — lever now holds true horizontal with 0.00 Hz oscillation and symmetric motor output. See `decision/DR-008-cascaded-pid.md`, `decision/DR-010-grv-calibrated-gyro-dual-report.md`.
- **Post-rebuild retuning (2026-04-07)** — Full PID retuning on new CF frame after mechanical rebuild invalidated all previous gains. Inverted tracking diagnosed (AXIS_CENTER 406→275). Force budget analysis: frame is precisely balanced; ~18g of resistance at −59° is wire tension + bearing friction (cables routed outside rotation axis, no slip ring). Found `angle_kp × start_error ≥ ANGLE_RATE_LIMIT` must hold for authority at start position. Systematic tuning: angle_kp 1.0→3.5, angle_kd 0.1→0.3, iterm_limit 30→100. New baseline (run `2026-04-07_16-19-21`): **6.90° HoldMAE, 1.3s reach, 124.8s hold, 0.05 Hz oscillation**. See `decision/DR-008-cascaded-pid.md` Amendment 2026-04-07.

**Current focus:** Post-rebuild PID retuning complete (2026-04-07). New baseline established: **6.90° HoldMAE, 1.3s time-to-reach, 124.8s hold time** (run `2026-04-07_16-19-21`). Next: thrust expo in `LeverMixer` to reduce near-setpoint oscillation and improve hold accuracy.

**Roadmap (see README.md for full details):**
- M5: Multi-axis control (depends on hardware evolution)

## Hardware Components

- **Raspberry Pi Pico 2** - Main microcontroller
- **BNO085 IMU** - 9-axis IMU with onboard sensor fusion, provides quaternion output
- **AS5600 Magnetic Encoder** - 12-bit absolute position encoder (reference sensor)
- **Pimoroni Pico Display Pack** - Buttons (GPIO 12–15) + RGB LED (GPIO 6/7/8) only; LCD disconnected to resolve SPI0 conflict (see DR-004). Operator interface implemented in `ui.py`.
- **2x Drone Motors + ESCs** - DShot protocol control via PIO
- **Adafruit Micro SD SPI Breakout** ([#4682](https://www.adafruit.com/product/4682)) - MicroSD via SPI0, 3V only
- **Adafruit PCF8523 RTC Breakout** ([#5189](https://www.adafruit.com/product/5189)) - RTC via I2C Bus 0 (shared with sensors), battery-backed; together provide black box telemetry logging (see DR-002)
- **Power Distribution Board** - Motor power supply

## Project Structure

```
├── src/                 # Authored flight control source (deployed to Pico)
│   ├── main.py          # Entry point - upload to Pico, runs on boot
│   ├── pid.py           # PID controller with anti-windup and term introspection
│   ├── mixer.py         # LeverMixer — differential thrust for 2-motor lever
│   ├── ui.py            # Operator interface — Pimoroni Display Pack buttons + RGB LED
│   └── telemetry/
│       ├── recorder.py  # TelemetryRecorder, PrintSink, SdSink, read_rtc
│       └── sdcard.py    # SD card SPI driver (micropython-lib, with stop bit fix)
├── AS5600/              # Git submodule: github.com/c0ffee2code/AS5600
│   └── driver/as5600.py
├── BNO085/              # Git submodule: github.com/c0ffee2code/BNO085
│   └── driver/
│       ├── bno08x.py
│       └── i2c.py
├── DShot/               # Git submodule: github.com/c0ffee2code/DShot
│   └── driver/
│       ├── dshot_pio.py
│       └── motor_throttle_group.py
├── .claude/
│   └── skills/
│       └── analyse-flight/  # Skill: full analysis pipeline for a single flight
│           ├── SKILL.md
│           ├── scripts/
│           │   ├── plot.py            # Step 1: 5-subplot diagnostic figure
│           │   ├── score_flight.py    # Step 2: pass/fail KPI gate
│           │   └── profile_flight.py  # Step 3: detailed statistical profile
│           └── templates/
│               └── flight_analysis.md  # Structured report template
├── tools/               # Pico utilities (upload to Pico, not used on desktop)
│   ├── set_rtc.py           # Set PCF8523 RTC clock
│   ├── coord_check.py       # Axis calibration after mechanical reassembly
│   ├── tare_and_measure.py  # IMU tare calibration with before/after comparison
│   └── bench_sweep.py       # Open-loop single-motor encoder sweep
├── test_runs/           # Copied run folders from SD card for analysis
│   └── flights/
│       └── YYYY-MM-DD_hh-mm-ss/  # One folder per run
│           ├── config.json  # System settings snapshot (PID gains, motor limits, etc.)
│           └── log.csv      # Telemetry CSV
├── decision/            # Architecture Decision Records
└── resources/           # Docs, datasheets
```

**Deployment:** Upload the following files to Pico root (flat structure). Use the deploy skill (`--full` for source changes, no flag for config-only):
- `src/config.json`
- `src/main.py`
- `src/pid.py`
- `src/mixer.py`
- `src/ui.py`
- `src/telemetry/recorder.py` (deployed as `recorder.py`)
- `src/telemetry/sdcard.py` (deployed as `sdcard.py`)
- `AS5600/driver/as5600.py`
- `BNO085/driver/bno08x.py` + `BNO085/driver/i2c.py`
- `DShot/driver/dshot_pio.py` + `DShot/driver/motor_throttle_group.py`

## Architecture

### Telemetry (`src/telemetry/`)

- `recorder.py` — Contains the full telemetry pipeline:
  - `TelemetryRecorder` — facade called from main loop, handles decimation and CSV formatting, delegates I/O to a pluggable sink. `begin_session()` resets the counter and emits the CSV header. CSV format: `T_MS,ENC_QR,ENC_QI,ENC_QJ,ENC_QK,IMU_QR,IMU_QI,IMU_QJ,IMU_QK,GYRO_X,ANG_ERR,ANG_P,ANG_I,ANG_D,RATE_SP,RATE_ERR,RATE_P,RATE_I,RATE_D,PID_OUT,M1,M2`. Quaternion values at 5 decimal places.
  - `PrintSink` — prints CSV rows to REPL serial console.
  - `SdSink` — owns the full SD card lifecycle (SPI init, mount, RTC read, directory create, write, unmount). Creates a folder-per-run directory (`/sd/flights/YYYY-MM-DD_hh-mm-ss/`) containing `log.csv` and a raw binary copy of `config.json` from the Pico root.
  - `read_rtc(i2c, addr=0x68)` — one-shot read of PCF8523 RTC given an already-constructed I2C object. Used by `SdSink` for directory naming.
- `sdcard.py` — SD card SPI driver from micropython-lib with a stop bit fix (`crc | 0x01`). The upstream driver omits the mandatory end bit in the SPI command frame, which causes some cards to reject all commands after CMD8.

### AS5600 Encoder (`AS5600/driver/as5600.py`)

Magnetic rotary encoder driver. Key function: `to_degrees(raw_angle, axis_center)` converts raw 12-bit readings to degrees relative to a calibrated center position. Includes low-latency filter configuration and diagnostic telemetry.

### BNO085 IMU (`BNO085/driver/`)

- `bno08x.py` - BNO08x driver with SHTP protocol, interrupt-driven sensor updates, quaternion/euler output, and precise timestamp tracking.
- `i2c.py` - I2C transport layer for BNO08x. Handles non-standard clock stretching and fragment reassembly.

### Operator Interface (`src/ui.py`)

Hardware: [Pimoroni Pico Display Pack](https://shop.pimoroni.com/products/pico-display-pack). Buttons on GPIO 12–15 and RGB LED on GPIO 6/7/8. LCD is disconnected (SPI0 conflict with SD card breakout, see DR-004). Status indicated by LED color: blue=idle, green=armed, red=error. `ui.py` owns all LED/button pin constants, hardware init, and UI helpers (`set_led`, `buttons_by_held`, `wait_for_arm`, `wait_for_go`).

### Motor Control (`DShot/driver/`)

- `dshot_pio.py` - Low-level DShot protocol via RP2040/RP2350 PIO. Supports DSHOT150/300/600/1200.
- `motor_throttle_group.py` - Dual-core facade for managing multiple motors. Core 1 runs a dedicated 1kHz command loop for reliable ESC communication. Provides arming, throttle control, emergency stop, and health monitoring.

### Standard Test Convention

Each standardised run starts with **M1 end down** (lever resting on the restrictor). The algorithm must lift the lever to within ±10° of horizontal (0°) and hold it there. KPIs measured from this starting condition:

- **Reached**: did the encoder enter ±10° at any point?
- **T→0**: seconds from run start to first entry into ±10°
- **HoldMAE**: encoder MAE from first reach to end of run
- **T@0**: total seconds spent within ±10°

Re-tare the IMU at the start of each session using the precision jig + bubble level (jig-measured residual: ~0.10°).

### Folder-per-run Convention

Each stabilisation session creates a timestamped directory on the SD card:
```
/sd/flights/YYYY-MM-DD_hh-mm-ss/
    config.json    # Raw copy of config.json from Pico root (source of truth)
    log.csv        # Telemetry CSV
```

`config.json` is uploaded to the Pico before a run (via the deploy skill) and copied as-is to the run folder by `SdSink.init_session()`. It is the single source of truth — no serialisation from Python objects. Top-level structure: `vehicle` (imu, angle_pid, rate_pid, motor, feedforward — algorithm parameters that stay fixed across sessions), `bench` (encoder, session — rig-specific; session contains duration_s and setpoint.{roll_deg, pitch_deg, yaw_deg}), `telemetry` (sample_every). Parsed with `json` on desktop.

## Motor and Encoder Sign Convention

Motors are mounted such that **thrust pushes the motor end DOWN** (confirmed by single-motor test 2026-04-06: M1 alone → M1 end descended to restrictor).

| Motor | GPIO | Encoder at lowest position |
|-------|------|---------------------------|
| M1    | 10   | +58°                      |
| M2    | 11   | −59°                      |

- **Positive encoder angle** → M1 side is lower, M2 side is higher
- **Negative encoder angle** → M2 side is lower, M1 side is higher
- To correct M1 being low (positive angle): increase M2 throttle → M2 pushes down → lever pivots → M1 rises
- The error computation in `flight.py` (`feedforward_roll = setpoint_roll_deg - (imu_roll + gyro_x * lead_s)`) implements this correctly; at setpoint=0° this reduces to `-(imu_roll + ...)`

## Key Constants

- `AXIS_CENTER` in `src/main.py` - Encoder offset for horizontal lever position (recalibrate when mechanical setup changes)
- `INNER_INTERVAL_MS` in `src/main.py` - Inner (rate) loop period (5ms = 200 Hz)
- `OUTER_INTERVAL_TICKS` in `src/main.py` - Outer (angle) loop runs every Nth inner cycle (4 = 50 Hz)
- `IMU_REPORT_HZ` in `src/main.py` - Calibrated gyroscope report rate (200 Hz, matches inner loop)
- `GRV_REPORT_HZ` in `src/main.py` - Game rotation vector report rate (50 Hz, matches outer loop)
- `FEEDFORWARD_LEAD_MS` in `src/main.py` - Feedforward lead time compensating GRV filter lag (see DR-006)

## I2C Addresses

- AS5600 encoder: `0x36`
- BNO085 IMU: `0x4a`
- PCF8523 RTC: `0x68`

## Pin Assignments

All pin constants are defined in `src/main.py` with descriptive names:

| GPIO | Constant | Function |
|------|----------|----------|
| 0 | `PIN_I2C0_SDA` | I2C bus 0 SDA — sensors (AS5600 + BNO085 + PCF8523 RTC) |
| 1 | `PIN_I2C0_SCL` | I2C bus 0 SCL |
| 2 | `PIN_IMU_RST` | BNO085 reset |
| 3 | `PIN_IMU_INT` | BNO085 interrupt |
| 6 | `PIN_LED_R` | RGB LED Red — error |
| 7 | `PIN_LED_G` | RGB LED Green — armed/stabilizing |
| 8 | `PIN_LED_B` | RGB LED Blue — ready to arm |
| 10 | `PIN_MOTOR1` | Motor 1 DShot |
| 11 | `PIN_MOTOR2` | Motor 2 DShot |
| 12 | `PIN_BTN_A` | Button A |
| 13 | `PIN_BTN_B` | Button B |
| 14 | `PIN_BTN_X` | Button X |
| 15 | `PIN_BTN_Y` | Button Y |
| 16 | `PIN_SD_MISO` | SD card breakout MISO (SPI0) |
| 17 | `PIN_SD_CS` | SD card breakout CS (SPI0) |
| 18 | `PIN_SD_SCK` | SD card breakout SCK (SPI0) |
| 19 | `PIN_SD_MOSI` | SD card breakout MOSI (SPI0) |

**Note:** PCF8523 RTC shares I2C Bus 0 (GPIO 0/1) with AS5600 and BNO085. No address conflicts (0x68 vs 0x36/0x4A). RTC is read once per session before the control loop starts, so there is no bus contention at runtime.