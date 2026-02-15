# Flight Benchy

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. Experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

## Hardware

- **Raspberry Pi Pico 2** — Main microcontroller (RP2350, dual-core)
- **AS5600 Magnetic Encoder** — 12-bit absolute position at pivot (ground truth reference, ~0.088° resolution)
- **BNO085 IMU** — 9-axis IMU with onboard sensor fusion (primary control input, game rotation vector)
- **2x Drone Motors + ESCs** — DShot600 protocol via PIO, mounted on opposite ends of lever
- **Pimoroni Pico Display Pack** — Buttons + RGB LED only; LCD disconnected (see [ADR-004](decision/ADR-004-operator-interface.md))
- **Adafruit PiCowbell Adalogger** — PCF8523 RTC + MicroSD for black box telemetry logging
- **Power Distribution Board** — Motor power supply

## Mechanical Setup

Metal frame with 3D-printed elements forming a swinging lever pivoting around a central axis. Two drone motors on opposite ends produce **downward** thrust (inverted for safety — bench can't fly off the desk). Differential thrust creates torque to control lever angle. Mechanical range is approximately ±50°.

## Roadmap

### M1: Single-axis PID with encoder — DONE

Single PI(D) loop at 50 Hz using AS5600 encoder feedback. Differential thrust mixer for 2 motors. Lever holds at 0° within ±3°. See [ADR-001](decision/ADR-001-pid-lever-stabilization.md).

### M2: Switch to BNO085 IMU as primary control input — DONE

PID input switched from AS5600 encoder to BNO085 game rotation vector (gyro+accel, no magnetometer). Roll angle extracted from quaternion via single `atan2`. AS5600 encoder retained as telemetry-only ground truth. Telemetry now stores raw quaternions from both sensors for offline analysis. See [ADR-005](decision/ADR-005-bno085-pid-input.md).

### M2a: Telemetry logging (black box) — DONE

Timestamped CSV logging to SD card via Adalogger PiCowbell. Each run creates a folder (`/sd/blackbox/YYYY-MM-DD_hh-mm-ss/`) containing `log.csv` and `config.yaml` (system settings snapshot). `ticks_ms` provides precise row timing. See [ADR-002](decision/ADR-002-telemetry-logging.md).

Motor pins reassigned from GPIO 4/5 → 6/7 → 10/11 (freeing I2C for Adalogger RTC, then RGB LED pins).

### M3: Mixer abstraction — DONE

Extracted `base ± output` motor mapping into `LeverMixer` class (`mixer.py`). Makes code drone-topology-agnostic — swap a mix table to support different frame types (2-motor lever, quadcopter X-frame, etc.). Also reorganized telemetry into `telemetry/` package.

**Depends on:** M1 (pure code refactor, no hardware dependency).

### M4: Cascaded PID (angle loop + rate loop)

Replace single angle PID with two nested loops: an outer angle loop (~50-100 Hz) feeding desired rotation rate to an inner rate loop (~500+ Hz) using raw gyro data. This is how real flight controllers (Betaflight, ArduPilot) work — the inner loop uses near-zero-lag gyro data for crisp response.

**Depends on:** M2 (IMU as input), M2a (telemetry to validate improvement), M3 (clean mixer).

### M5: Multi-axis control

Add roll and/or yaw axes. Requires either mechanical modifications to the bench or moving to an actual drone frame.

**Depends on:** M3 (mixer), M4 (cascaded PID), hardware evolution.

## Test Bench vs Real Drone

See [ADR-001, "Test Bench vs Real Drone" section](decision/ADR-001-pid-lever-stabilization.md) for a detailed comparison covering: single axis vs three axes, single PID vs cascaded PIDs, encoder vs IMU, and fixed pivot vs free flight.

## Project Structure

```
├── main.py              # Entry point — upload to Pico, runs on boot
├── pid.py               # PID controller with anti-windup and term introspection
├── mixer.py             # LeverMixer — differential thrust for 2-motor lever
├── telemetry/
│   ├── recorder.py      # TelemetryRecorder, PrintSink, SdSink, read_rtc
│   └── sdcard.py        # SD card SPI driver (micropython-lib, with stop bit fix)
├── AS5600/              # Git submodule: github.com/c0ffee2code/AS5600
├── BNO085/              # Git submodule: github.com/c0ffee2code/BNO085
├── DShot/               # Git submodule: github.com/c0ffee2code/DShot
├── tools/
│   └── analyse_telemetry.py  # Desktop telemetry analyser (matplotlib + numpy)
├── test_runs/           # Copied run folders from SD card for offline analysis
├── pimoroni/            # Display driver (not deployed — LCD disconnected)
├── decision/            # Architecture Decision Records
└── resources/           # Datasheets, protocol docs
```

## Deployment

Clone with submodules:
```bash
git clone --recurse-submodules <repo-url>
```

Upload to Pico root (flat structure):
- `main.py`
- `pid.py`
- `mixer.py`
- `telemetry/recorder.py` (as `recorder.py`)
- `telemetry/sdcard.py` (as `sdcard.py`)
- `AS5600/driver/as5600.py`
- `BNO085/driver/bno08x.py` + `i2c.py`
- `DShot/driver/dshot_pio.py` + `motor_throttle_group.py`

## License

See individual submodule repositories for their licenses.