# Flight Benchy

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. Experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

## Hardware

- **Raspberry Pi Pico 2** — Main microcontroller (RP2350, dual-core)
- **AS5600 Magnetic Encoder** — 12-bit absolute position at pivot (ground truth reference, ~0.088° resolution)
- **BNO085 IMU** — 9-axis IMU with onboard sensor fusion; game rotation vector (GRV, 0x08) for outer angle loop, calibrated gyroscope (0x02) for inner rate loop
- **2x Drone Motors + ESCs** — DShot600 protocol via PIO, mounted on opposite ends of lever
- **Pimoroni Pico Display Pack** — Buttons + RGB LED only; LCD disconnected (see [ADR-004](decision/ADR-004-operator-interface.md))
- **Adafruit Micro SD SPI Breakout** ([#4682](https://www.adafruit.com/product/4682)) + **Adafruit PCF8523 RTC Breakout** ([#5189](https://www.adafruit.com/product/5189)) — MicroSD + RTC for black box telemetry logging
- **Power Distribution Board** — Motor power supply

## Mechanical Setup

Metal frame with 3D-printed elements forming a swinging lever pivoting around a central axis. Two drone motors on opposite ends produce **downward** thrust (inverted for safety — bench can't fly off the desk). Differential thrust creates torque to control lever angle. Mechanical range is approximately ±50°.

## Roadmap

### M1: Single-axis PID with encoder — DONE

Single PI(D) loop at 50 Hz using AS5600 encoder feedback. Differential thrust mixer for 2 motors. Lever holds at 0° within ±3°. See [ADR-001](decision/ADR-001-pid-lever-stabilization.md).

### M2: Switch to BNO085 IMU as primary control input — DONE

PID input switched from AS5600 encoder to BNO085 game rotation vector (gyro+accel, no magnetometer). Roll angle extracted from quaternion via single `atan2`. AS5600 encoder retained as telemetry-only ground truth. Telemetry now stores raw quaternions from both sensors for offline analysis. See [ADR-005](decision/ADR-005-bno085-pid-input.md).

BNO085 sensor calibration (accel, gyro, magnetometer) and all-axes tare completed and saved to flash. Calibration + tare reduced MAE from 22 deg to 2.87 deg and eliminated limit cycle oscillation. See [BNO085 ADR-004](BNO085/decision/004-sensor-calibration.md).

### M2a: Telemetry logging (black box) — DONE

Timestamped CSV logging to SD card via SD card breakout + PCF8523 RTC breakout. Each run creates a folder (`/sd/blackbox/YYYY-MM-DD_hh-mm-ss/`) containing `log.csv` and `config.yaml` (system settings snapshot). `ticks_ms` provides precise row timing. See [ADR-002](decision/ADR-002-telemetry-logging.md).

Motor pins reassigned from GPIO 4/5 → 6/7 → 10/11 (freeing GPIO 4/5 for PCF8523 RTC, then RGB LED pins).

### M3: Mixer abstraction — DONE

Extracted `base ± output` motor mapping into `LeverMixer` class (`mixer.py`). Makes code drone-topology-agnostic — swap a mix table to support different frame types (2-motor lever, quadcopter X-frame, etc.). Also reorganized telemetry into `telemetry/` package.

**Depends on:** M1 (pure code refactor, no hardware dependency).

### M4: Cascaded PID (angle loop + rate loop) — DONE

Two nested PID loops replacing the single angle PID. Outer angle loop (50 Hz) computes a desired rotation rate from GRV quaternion (game rotation vector, drift-free); inner rate loop (200 Hz) tracks that rate using calibrated gyroscope angular velocity. Initial sensor choice (GIRV) was replaced after hardware testing revealed ~1.5°/min gyro integration drift; see [ADR-010](decision/ADR-010-grv-calibrated-gyro-dual-report.md). Both loops run in a single main loop with iteration-counter gating.

Baseline result (2026-02-22, 77s run with active disturbance): **0.36° MAE**, 0.09 Hz oscillation frequency, zero windup events, Pearson r=0.999. 3× improvement over prior baseline (1.12° MAE, 2026-02-20) following BNO085 re-calibration, correct tare procedure, and AXIS_CENTER correction (422→411→406).

Post-baseline improvements (2026-02-22): precision 3D-printed jig established true AXIS_CENTER=406, lever mechanically balanced, `ki=0.05` with `iterm_limit=5` added to angle PID to compensate for residual imbalance and slow GRV drift. Result: **0.00 Hz oscillation**, symmetric motor output, lever holds true horizontal.

See [ADR-008](decision/ADR-008-cascaded-pid.md) and [ADR-010](decision/ADR-010-grv-calibrated-gyro-dual-report.md).

**Depends on:** M2 (IMU as input), M2a (telemetry to validate improvement), M3 (clean mixer).

### Mechanical rebuild: aluminum lever → 3D-printed frame

Replacing 150 g aluminum lever with 12 g 3D-printed drone frame (~12× inertia reduction). Requires:
- Full PID retuning — current gains will be too aggressive; start kp at ~25–30% of current values
- AXIS_CENTER recalibration with precision jig
- Fresh baseline test run to re-establish MAE reference

**Depends on:** hardware fabrication.

### M5: Multi-axis control

Add roll and/or yaw axes. Requires either mechanical modifications to the bench or moving to an actual drone frame.

**Depends on:** M3 (mixer), M4 (cascaded PID), mechanical rebuild.

## Known Issues / Backlog

### BNO085 intermittent I2C EIO crash

Rare `OSError: [Errno 5] EIO` on `imu.update_sensors()` during a run. First captured 2026-02-23 (`ticks_ms=163752`, ~2 min 44 s into run). Likely cause: BNO085 internal firmware assert or watchdog reset leaving the I2C bus in an inconsistent state. Crash log written to `/crash.log` on onboard flash. Needs dedicated uptime / stability test runs to reproduce reliably before a fix is designed.

### ~~Rate PID D-term: switch to measurement derivative before enabling~~ — FIXED

`PID.compute` now accepts an optional `measurement` parameter. When provided, D is computed
from `−d(measurement)/dt` instead of `d(error)/dt`, avoiding derivative kick when the setpoint
steps. The inner rate loop passes `measurement=gyro_x`. The outer angle loop is unaffected
(setpoint is a constant 0°, so error derivative already equals measurement derivative).

### Thrust linearization

Motor thrust ∝ RPM², and RPM ≈ DShot throttle value, so effective thrust ∝ throttle². `LeverMixer`
outputs throttle values directly, meaning plant gain varies with operating point. Applying `sqrt()`
to `output` before the mixer maps PID output to a thrust-linear space, making gains consistent
across throttle levels. Low priority until throttle-dependent oscillation is observed.

### I-term relax for large disturbances

During large transients the angle PID I-term continues accumulating during nonlinear large-signal
operation. BetaFlight's "I-term relax" freezes integration when angular rate exceeds a threshold,
preventing windup during disturbances without needing a high `iterm_limit`. Low priority given
current `iterm_limit=5` already bounds windup, but revisit if disturbance response
characterization reveals systematic post-disturbance overshoot.

### Telemetry not flushed on crash

When an unhandled exception exits the control loop, `disarm()` is never reached and `telemetry.end_session()` is not called. The SD file is left open/unflushed; the last buffered rows may be lost. Fix: call `end_session()` from the crash path in `main()`, guarded so it does not raise if telemetry was never initialised.

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