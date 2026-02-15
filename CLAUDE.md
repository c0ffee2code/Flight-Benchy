# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. The goal is to experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

**Mechanical Setup:** Metal frame with 3D-printed elements forming a swinging lever. Two drone motors with ESCs mounted on the lever provide thrust for attitude control. A power distribution board supplies the motors.

**Sensor Strategy:**
- **AS5600 magnetic encoder** at the rotation center serves as ground truth reference due to its high precision (~0.09°) and near-instant position output
- **BNO085 IMU** provides the sensor input that a real flight controller would use (gyroscope, accelerometer, magnetometer with onboard sensor fusion)

## Development Approach

**Completed:**
- M1 — Single-axis PI(D) controller with AS5600 encoder, validated on hardware. Lever holds at 0° within ±3°. See `decision/ADR-001-pid-lever-stabilization.md`.
- M2 — BNO085 IMU as primary PID input (game rotation vector). AS5600 encoder is telemetry-only ground truth. Quaternion telemetry format. See `decision/ADR-005-bno085-pid-input.md`.
- M2a — Black box telemetry logging to SD card via Adalogger PiCowbell. RTC-timestamped filenames, `ticks_ms` row timing, CSV format. See `decision/ADR-002-telemetry-logging.md`.
- M3 — Mixer extraction (`LeverMixer` in `mixer.py`) + telemetry reorganization into `telemetry/` package.
- ADR-004 — Operator interface: LCD disconnected (resolves SPI0 conflict), buttons + RGB LED only. Motors moved to GPIO 10/11, RGB LED on GPIO 6/7/8. Standard MicroPython firmware.

**Current focus:** M4 — Cascaded PID (angle loop + rate loop using raw gyro).

**Roadmap (see README.md for full details):**
- M4: Cascaded PID — angle loop + rate loop using raw gyro (depends on M2, M2a, M3)
- M5: Multi-axis control (depends on hardware evolution)

## Hardware Components

- **Raspberry Pi Pico 2** - Main microcontroller
- **BNO085 IMU** - 9-axis IMU with onboard sensor fusion, provides quaternion output
- **AS5600 Magnetic Encoder** - 12-bit absolute position encoder (reference sensor)
- **Pimoroni Pico Display Pack** - Buttons (GPIO 12–15) + RGB LED (GPIO 6/7/8) only; LCD disconnected to resolve SPI0 conflict (see ADR-004)
- **2x Drone Motors + ESCs** - DShot protocol control via PIO
- **Adafruit PiCowbell Adalogger** - PCF8523 RTC + MicroSD for telemetry logging (see ADR-002)
- **Power Distribution Board** - Motor power supply

## Project Structure

```
├── main.py              # Entry point - upload to Pico, runs on boot
├── pid.py               # PID controller with anti-windup and term introspection
├── mixer.py             # LeverMixer — differential thrust for 2-motor lever
├── telemetry/
│   ├── recorder.py      # TelemetryRecorder, PrintSink, SdSink, read_rtc
│   └── sdcard.py        # SD card SPI driver (micropython-lib, with stop bit fix)
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
├── tools/
│   └── analyse_telemetry.py  # Desktop telemetry analyser (not deployed to Pico)
├── test_runs/           # Copied run folders from SD card for analysis
│   └── YYYY-MM-DD_hh-mm-ss/  # One folder per run
│       ├── config.yaml  # System settings snapshot (PID gains, motor limits, etc.)
│       └── log.csv      # Telemetry CSV
├── pimoroni/
│   └── pico_display_pack.py  # Display driver (NOT deployed — LCD disconnected, see ADR-004)
├── decision/            # Architecture Decision Records
└── resources/           # Docs, datasheets
```

**Deployment:** Upload the following files to Pico root (flat structure):
- `main.py`
- `pid.py`
- `mixer.py`
- `telemetry/recorder.py` (deployed as `recorder.py`)
- `telemetry/sdcard.py` (deployed as `sdcard.py`)
- `AS5600/driver/as5600.py`
- `BNO085/driver/bno08x.py` + `BNO085/driver/i2c.py`
- `DShot/driver/dshot_pio.py` + `DShot/driver/motor_throttle_group.py`

## Architecture

### Telemetry (`telemetry/`)

- `recorder.py` — Contains the full telemetry pipeline:
  - `TelemetryRecorder` — facade called from main loop, handles decimation and CSV formatting, delegates I/O to a pluggable sink. `begin_session(config=None)` accepts an optional YAML config string. CSV format: `T_MS,ENC_QR,ENC_QI,ENC_QJ,ENC_QK,IMU_QR,IMU_QI,IMU_QJ,IMU_QK,ERR,P,I,D,PID_OUT,M1,M2`. Quaternion values at 5 decimal places.
  - `PrintSink` — prints CSV rows to REPL serial console. No-op `write_config()`.
  - `SdSink` — owns the full SD card lifecycle (SPI init, mount, RTC read, directory create, write, unmount). Creates a folder-per-run directory (`/sd/blackbox/YYYY-MM-DD_hh-mm-ss/`) containing `log.csv` and `config.yaml`.
  - `read_rtc(sda, scl)` — one-shot SoftI2C read of PCF8523 RTC. Used by `SdSink` for directory naming.
- `sdcard.py` — SD card SPI driver from micropython-lib with a stop bit fix (`crc | 0x01`). The upstream driver omits the mandatory end bit in the SPI command frame, which causes some cards to reject all commands after CMD8.

### AS5600 Encoder (`AS5600/driver/as5600.py`)

Magnetic rotary encoder driver. Key function: `to_degrees(raw_angle, axis_center)` converts raw 12-bit readings to degrees relative to a calibrated center position. Includes low-latency filter configuration and diagnostic telemetry.

### BNO085 IMU (`BNO085/driver/`)

- `bno08x.py` - BNO08x driver with SHTP protocol, interrupt-driven sensor updates, quaternion/euler output, and precise timestamp tracking.
- `i2c.py` - I2C transport layer for BNO08x. Handles non-standard clock stretching and fragment reassembly.

### Operator Interface (buttons + RGB LED)

Buttons on GPIO 12–15 and RGB LED on GPIO 6/7/8 from the Pimoroni Display Pack. LCD is disconnected (SPI0 conflict with Adalogger SD card, see ADR-004). Status indicated by LED color: blue=idle, green=armed, red=error. `pimoroni/pico_display_pack.py` remains in repo for reference but is not deployed.

### Motor Control (`DShot/driver/`)

- `dshot_pio.py` - Low-level DShot protocol via RP2040/RP2350 PIO. Supports DSHOT150/300/600/1200.
- `motor_throttle_group.py` - Dual-core facade for managing multiple motors. Core 1 runs a dedicated 1kHz command loop for reliable ESC communication. Provides arming, throttle control, emergency stop, and health monitoring.

### Telemetry Analyser (`tools/analyse_telemetry.py`)

Desktop Python script (not deployed to Pico). Reads run folders from `test_runs/`, converts quaternions to roll angles offline, produces 4-subplot diagnostic figures and console statistics. Supports single-run analysis and two-run side-by-side comparison. Dependencies: `numpy`, `matplotlib`, `pyyaml`.

### Folder-per-run Convention

Each stabilisation session creates a timestamped directory on the SD card:
```
/sd/blackbox/YYYY-MM-DD_hh-mm-ss/
    config.yaml    # System settings (PID gains, motor limits, IMU rate, etc.)
    log.csv        # Telemetry CSV
```

`config.yaml` is written as plain string formatting on MicroPython (no YAML library) and parsed with `pyyaml` on desktop. Contains: `imu`, `pid`, `motor`, `encoder`, `telemetry` sections.

## Key Constants

- `AXIS_CENTER` in `main.py` - Encoder offset for horizontal lever position (recalibrate when mechanical setup changes)
- `IMU_REPORT_HZ` in `main.py` - BNO085 game rotation vector report rate (100 Hz, 2x PID rate)

## I2C Addresses

- AS5600 encoder: `0x36`
- BNO085 IMU: `0x4a`
- PCF8523 RTC: `0x68`

## Pin Assignments

All pin constants are defined in `main.py` with descriptive names:

| GPIO | Constant | Function |
|------|----------|----------|
| 0 | `PIN_I2C0_SDA` | I2C bus 0 SDA — sensors (AS5600 + BNO085) |
| 1 | `PIN_I2C0_SCL` | I2C bus 0 SCL |
| 2 | `PIN_IMU_RST` | BNO085 reset |
| 3 | `PIN_IMU_INT` | BNO085 interrupt |
| 4 | `PIN_RTC_SDA` | Adalogger RTC SoftI2C SDA |
| 5 | `PIN_RTC_SCL` | Adalogger RTC SoftI2C SCL |
| 6 | `PIN_LED_R` | RGB LED Red — error |
| 7 | `PIN_LED_G` | RGB LED Green — armed/stabilizing |
| 8 | `PIN_LED_B` | RGB LED Blue — ready to arm |
| 10 | `PIN_MOTOR1` | Motor 1 DShot |
| 11 | `PIN_MOTOR2` | Motor 2 DShot |
| 12 | `PIN_BTN_A` | Button A |
| 13 | `PIN_BTN_B` | Button B |
| 14 | `PIN_BTN_X` | Button X |
| 15 | `PIN_BTN_Y` | Button Y |
| 16 | `PIN_SD_MISO` | Adalogger SD card MISO (SPI0) |
| 17 | `PIN_SD_CS` | Adalogger SD card CS (SPI0) |
| 18 | `PIN_SD_SCK` | Adalogger SD card SCK (SPI0) |
| 19 | `PIN_SD_MOSI` | Adalogger SD card MOSI (SPI0) |

**Note:** GPIO 4/5 are I2C0 alternate pins on RP2350, not I2C1. The RTC uses SoftI2C (bit-banged) to avoid conflicting with the sensor I2C bus on GPIO 0/1. See ADR-002 for details.