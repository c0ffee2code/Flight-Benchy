# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. The goal is to experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

**Mechanical Setup:** Metal frame with 3D-printed elements forming a swinging lever. Two drone motors with ESCs mounted on the lever provide thrust for attitude control. A power distribution board supplies the motors.

**Sensor Strategy:**
- **AS5600 magnetic encoder** at the rotation center serves as ground truth reference due to its high precision (~0.09°) and near-instant position output
- **BNO085 IMU** provides the sensor input that a real flight controller would use (gyroscope, accelerometer, magnetometer with onboard sensor fusion)

## Development Approach

**Current milestone:** Characterize BNO085 sensor performance by measuring angle error and time lag against the AS5600 reference. This data is critical because a flight controller must compensate for sensor lag—controlling based on "where you are now" with a delayed sensor leads to instability and crashes. The control system needs to predict where it *will be* and adjust thrust accordingly.

**Future milestones:**
- Telemetry logging from both sensors (encoder + IMU)
- Log lag and angle difference over time for troubleshooting and debugging
- Implement control loops with lag compensation
- Implement bidirectional DShot for ESC telemetry (RPM, voltage, temperature)
- Analyze motor response lag (time between throttle command and RPM reaching target)

## Hardware Components

- **Raspberry Pi Pico 2** - Main microcontroller
- **BNO085 IMU** - 9-axis IMU with onboard sensor fusion, provides quaternion output
- **AS5600 Magnetic Encoder** - 12-bit absolute position encoder (reference sensor)
- **Pimoroni Pico Display Pack** - 240x135 LCD for real-time data visualization
- **2x Drone Motors + ESCs** - DShot protocol control via PIO
- **Power Distribution Board** - Motor power supply

## Project Structure

```
├── main.py              # Entry point - upload to Pico, runs on boot
├── adafruit/            # Sensor drivers (upload to Pico)
├── pimoroni/            # Display driver (upload to Pico)
├── dshot/               # Motor control (upload to Pico)
├── examples/            # Test scripts - do NOT upload to Pico
└── resources/           # Docs, datasheets
```

**Deployment:** Upload `main.py` + `adafruit/` + `pimoroni/` + `dshot/` to Pico. The `examples/` folder stays on your PC.

**Running examples:** Temporarily copy an example script to Pico as `main.py`, or run directly via Thonny/mpremote.

## Architecture

### Sensor Drivers (`adafruit/`)

- `adafruit/encoder/as5600.py` - AS5600 driver. Key function: `to_degrees(raw_angle, axis_center)` converts raw 12-bit readings to degrees relative to a calibrated center position.
- `adafruit/imu/bradcar/bno08x.py` - BNO08x driver adapted from Adafruit. Uses interrupt-driven sensor updates with precise timestamp tracking.
- `adafruit/imu/bradcar/i2c.py` - I2C transport layer for BNO08x.

### Display (`pimoroni/display/`)

- `display_pack.py` - Display abstraction using PicoGraphics library.

### Motor Control (`dshot/`)

- `dshot/jrddupont/dshot_pio.py` - DShot protocol via RP2040 PIO. Supports DSHOT150/300/600/1200.
- See `resources/dshot_protocol.md` for full protocol specification (packet structure, timing, special commands, bidirectional DShot, telemetry).

### Examples (`examples/`)

Small standalone scripts for testing individual sensors, calibration routines, and experiments. Not part of the main application.

## Key Constants

- `AXIS_CENTER` in `main.py` - Encoder offset for horizontal lever position (recalibrate when mechanical setup changes)
- I2C: SCL=Pin 1, SDA=Pin 0, 400kHz

## I2C Addresses

- AS5600 encoder: `0x36`
- BNO085 IMU: `0x4a`

## Pin Assignments

- BNO085 reset: Pin 2
- BNO085 interrupt: Pin 3
- Motor 1 (DShot): Pin 4
- Motor 2 (DShot): Pin 5
- Display buttons: Pins 12 (A), 13 (B), 14 (X), 15 (Y)
