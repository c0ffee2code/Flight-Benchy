# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. The goal is to experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

**Mechanical Setup:** Metal frame with 3D-printed elements forming a swinging lever. Two drone motors with ESCs mounted on the lever provide thrust for attitude control. A power distribution board supplies the motors.

**Sensor Strategy:**
- **AS5600 magnetic encoder** at the rotation center serves as ground truth reference due to its high precision (~0.09°) and near-instant position output
- **BNO085 IMU** provides the sensor input that a real flight controller would use (gyroscope, accelerometer, magnetometer with onboard sensor fusion)

## Development Approach

**Completed:** M1 — Single-axis PI(D) controller with AS5600 encoder, validated on hardware. Lever holds at 0° within ±3°. See `decision/ADR-001-pid-lever-stabilization.md`.

**Current focus:** M2 — Switch PID input from AS5600 to BNO085 IMU. The IMU will be the primary and only control input (as on a real drone). AS5600 becomes telemetry-only ground truth for measuring IMU lag and angle error.

**Roadmap (see README.md for full details):**
- M2: BNO085 as primary control input (depends on driver work)
- M2a: Telemetry logging via Adalogger PiCowbell — RTC timestamps + SD card black box (see `decision/ADR-002-telemetry-logging.md`)
- M3: Mixer abstraction (pure refactor)
- M4: Cascaded PID — angle loop + rate loop using raw gyro (depends on M2, M2a, M3)
- M5: Multi-axis control (depends on hardware evolution)

## Hardware Components

- **Raspberry Pi Pico 2** - Main microcontroller
- **BNO085 IMU** - 9-axis IMU with onboard sensor fusion, provides quaternion output
- **AS5600 Magnetic Encoder** - 12-bit absolute position encoder (reference sensor)
- **Pimoroni Pico Display Pack** - 240x135 LCD for real-time data visualization
- **2x Drone Motors + ESCs** - DShot protocol control via PIO
- **Adafruit PiCowbell Adalogger** - PCF8523 RTC + MicroSD for telemetry logging (planned, see ADR-002)
- **Power Distribution Board** - Motor power supply

## Project Structure

```
├── main.py              # Entry point - upload to Pico, runs on boot
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
- `AS5600/driver/as5600.py`
- `BNO085/driver/bno08x.py` + `BNO085/driver/i2c.py`
- `DShot/driver/dshot_pio.py` + `DShot/driver/motor_throttle_group.py`
- `pimoroni/display/display_pack.py`

All driver files are uploaded flat to Pico root so MicroPython resolves imports without `sys.path` manipulation.

## Architecture

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
- I2C: SCL=Pin 1, SDA=Pin 0, 400kHz

## I2C Addresses

- AS5600 encoder: `0x36`
- BNO085 IMU: `0x4a`

## Pin Assignments

- BNO085 reset: Pin 2
- BNO085 interrupt: Pin 3
- Motor 1 (DShot): Pin 4 (will move to Pin 6 when Adalogger is integrated — see ADR-002)
- Motor 2 (DShot): Pin 5 (will move to Pin 7 when Adalogger is integrated — see ADR-002)
- Display buttons: Pins 12 (A), 13 (B), 14 (X), 15 (Y)
- Adalogger SD card (planned): MOSI=Pin 19, MISO=Pin 16, SCK=Pin 18, CS=Pin 17
- Adalogger RTC I2C (planned): SDA=Pin 4, SCL=Pin 5 (I2C bus 1)
