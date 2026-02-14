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
- M2a — Black box telemetry logging to SD card via Adalogger PiCowbell. RTC-timestamped filenames, `ticks_ms` row timing, CSV format. See `decision/ADR-002-telemetry-logging.md`.
- M3 — Mixer extraction (`LeverMixer` in `mixer.py`) + telemetry reorganization into `telemetry/` package.

**Current focus:** M2 — Switch PID input from AS5600 to BNO085 IMU. The IMU will be the primary and only control input (as on a real drone). AS5600 becomes telemetry-only ground truth for measuring IMU lag and angle error.

**Roadmap (see README.md for full details):**
- M2: BNO085 as primary control input (depends on driver work)
- M4: Cascaded PID — angle loop + rate loop using raw gyro (depends on M2, M2a, M3)
- M5: Multi-axis control (depends on hardware evolution)

## Hardware Components

- **Raspberry Pi Pico 2** - Main microcontroller
- **BNO085 IMU** - 9-axis IMU with onboard sensor fusion, provides quaternion output
- **AS5600 Magnetic Encoder** - 12-bit absolute position encoder (reference sensor)
- **Pimoroni Pico Display Pack** - 240x135 LCD for real-time data visualization
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
├── pimoroni/            # Display driver (upload to Pico)
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
- `pimoroni/display/display_pack.py`

## Architecture

### Telemetry (`telemetry/`)

- `recorder.py` — Contains the full telemetry pipeline:
  - `TelemetryRecorder` — facade called from main loop, handles decimation and CSV formatting, delegates I/O to a pluggable sink.
  - `PrintSink` — prints CSV rows to REPL serial console.
  - `SdSink` — owns the full SD card lifecycle (SPI init, mount, RTC read, file create, write, unmount). Constructed with pin numbers; encapsulates all Adalogger hardware interaction.
  - `read_rtc(sda, scl)` — one-shot SoftI2C read of PCF8523 RTC. Used by `SdSink` for filename generation.
- `sdcard.py` — SD card SPI driver from micropython-lib with a stop bit fix (`crc | 0x01`). The upstream driver omits the mandatory end bit in the SPI command frame, which causes some cards to reject all commands after CMD8.

### AS5600 Encoder (`AS5600/driver/as5600.py`)

Magnetic rotary encoder driver. Key function: `to_degrees(raw_angle, axis_center)` converts raw 12-bit readings to degrees relative to a calibrated center position. Includes low-latency filter configuration and diagnostic telemetry.

### BNO085 IMU (`BNO085/driver/`)

- `bno08x.py` - BNO08x driver with SHTP protocol, interrupt-driven sensor updates, quaternion/euler output, and precise timestamp tracking.
- `i2c.py` - I2C transport layer for BNO08x. Handles non-standard clock stretching and fragment reassembly.

### Display (`pimoroni/display/`)

- `display_pack.py` - Display abstraction using PicoGraphics library.

### Motor Control (`DShot/driver/`)

- `dshot_pio.py` - Low-level DShot protocol via RP2040/RP2350 PIO. Supports DSHOT150/300/600/1200.
- `motor_throttle_group.py` - Dual-core facade for managing multiple motors. Core 1 runs a dedicated 1kHz command loop for reliable ESC communication. Provides arming, throttle control, emergency stop, and health monitoring.

## Key Constants

- `AXIS_CENTER` in `main.py` - Encoder offset for horizontal lever position (recalibrate when mechanical setup changes)

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
| 6 | `PIN_MOTOR1` | Motor 1 DShot |
| 7 | `PIN_MOTOR2` | Motor 2 DShot |
| 12 | `PIN_BTN_A` | Display button A |
| 13 | `PIN_BTN_B` | Display button B |
| 14 | `PIN_BTN_X` | Display button X |
| 15 | `PIN_BTN_Y` | Display button Y |
| 16 | `PIN_SD_MISO` | Adalogger SD card MISO (SPI0) |
| 17 | `PIN_SD_CS` | Adalogger SD card CS (SPI0) |
| 18 | `PIN_SD_SCK` | Adalogger SD card SCK (SPI0) |
| 19 | `PIN_SD_MOSI` | Adalogger SD card MOSI (SPI0) |

**Note:** GPIO 4/5 are I2C0 alternate pins on RP2350, not I2C1. The RTC uses SoftI2C (bit-banged) to avoid conflicting with the sensor I2C bus on GPIO 0/1. See ADR-002 for details.