# Flight Benchy

Test bench for learning flight control systems, built around a Raspberry Pi Pico 2. Experiment with sensor fusion, control loops, and motor control in a controlled single-axis environment before applying concepts to real drones.

## Hardware

- **Raspberry Pi Pico 2** — Main microcontroller (RP2350, dual-core)
- **AS5600 Magnetic Encoder** — 12-bit absolute position at pivot (ground truth reference, ~0.088° resolution)
- **BNO085 IMU** — 9-axis IMU with onboard sensor fusion; game rotation vector (GRV, 0x08) for outer angle loop, calibrated gyroscope (0x02) for inner rate loop
- **2x Drone Motors + ESCs** — DShot600 protocol via PIO, mounted on opposite ends of lever
- **Pimoroni Pico Display Pack** — Buttons + RGB LED only; LCD disconnected (see [DR-004](decision/DR-004-operator-interface.md))
- **Adafruit Micro SD SPI Breakout** ([#4682](https://www.adafruit.com/product/4682)) + **Adafruit PCF8523 RTC Breakout** ([#5189](https://www.adafruit.com/product/5189)) — MicroSD + RTC for black box telemetry logging
- **Power Distribution Board** — Motor power supply

## Mechanical Setup

Carbon fiber drone frame (10 g) on a 3D-printed PETG subframe (8 g) forming a swinging lever pivoting around a central axis. Motor shaft separation: **6.5 cm**. Frame has 4 motor mounts; currently 2 motors installed (one each end), 2 more planned. Two drone motors produce **downward** thrust (inverted for safety — bench can't fly off the desk). Differential thrust creates torque to control lever angle. Mechanical range is approximately ±50°.

**Previous frame (retired 2026-04):** 125 g aluminum profile + PLA parts, 18.5 cm motor separation. ~7× heavier, ~3× longer arm.

## Roadmap

### M1: Single-axis PID with encoder — DONE

Single PI(D) loop at 50 Hz using AS5600 encoder feedback. Differential thrust mixer for 2 motors. Lever holds at 0° within ±3°. See [DR-001](decision/DR-001-pid-lever-stabilization.md).

### M2: Switch to BNO085 IMU as primary control input — DONE

PID input switched from AS5600 encoder to BNO085 game rotation vector (gyro+accel, no magnetometer). Roll angle extracted from quaternion via single `atan2`. AS5600 encoder retained as telemetry-only ground truth. Telemetry now stores raw quaternions from both sensors for offline analysis. See [DR-005](decision/DR-005-bno085-pid-input.md).

BNO085 sensor calibration (accel, gyro, magnetometer) and all-axes tare completed and saved to flash. Calibration + tare reduced MAE from 22 deg to 2.87 deg and eliminated limit cycle oscillation. See [BNO085 ADR-004](BNO085/decision/004-sensor-calibration.md).

### M2a: Telemetry logging (black box) — DONE

Timestamped CSV logging to SD card via SD card breakout + PCF8523 RTC breakout. Each run creates a folder (`/sd/flights/YYYY-MM-DD_hh-mm-ss/`) containing `log.csv` and `config.json` (raw copy of the deployed config — source of truth for analysis). `ticks_ms` provides precise row timing. See [DR-002](decision/DR-002-telemetry-logging.md).

Motor pins reassigned from GPIO 4/5 → 6/7 → 10/11 (freeing GPIO 4/5 for PCF8523 RTC, then RGB LED pins).

### M3: Mixer abstraction — DONE

Extracted `base ± output` motor mapping into `LeverMixer` class (`mixer.py`). Makes code drone-topology-agnostic — swap a mix table to support different frame types (2-motor lever, quadcopter X-frame, etc.). Also reorganized telemetry into `telemetry/` package.

**Depends on:** M1 (pure code refactor, no hardware dependency).

### M4: Cascaded PID (angle loop + rate loop) — DONE

Two nested PID loops replacing the single angle PID. Outer angle loop (50 Hz) computes a desired rotation rate from GRV quaternion (game rotation vector, drift-free); inner rate loop (200 Hz) tracks that rate using calibrated gyroscope angular velocity. Initial sensor choice (GIRV) was replaced after hardware testing revealed ~1.5°/min gyro integration drift; see [DR-010](decision/DR-010-grv-calibrated-gyro-dual-report.md). Both loops run in a single main loop with iteration-counter gating.

Baseline result (2026-02-22, 77s run with active disturbance): **0.36° MAE**, 0.09 Hz oscillation frequency, zero windup events, Pearson r=0.999. 3× improvement over prior baseline (1.12° MAE, 2026-02-20) following BNO085 re-calibration, correct tare procedure, and AXIS_CENTER correction (422→411→406).

Post-baseline improvements (2026-02-22): precision 3D-printed jig established true AXIS_CENTER=406, lever mechanically balanced, `ki=0.05` with `iterm_limit=5` added to angle PID to compensate for residual imbalance and slow GRV drift. Result: **0.00 Hz oscillation**, symmetric motor output, lever holds true horizontal.

See [DR-008](decision/DR-008-cascaded-pid.md) and [DR-010](decision/DR-010-grv-calibrated-gyro-dual-report.md).

**Depends on:** M2 (IMU as input), M2a (telemetry to validate improvement), M3 (clean mixer).

### Mechanical rebuild: aluminum lever → carbon fiber drone frame — DONE (2026-04-03)

Replaced 125 g aluminum profile + PLA parts (18.5 cm arm) with a 10 g carbon fiber drone frame + 8 g PETG subframe (6.5 cm motor separation). ~7× mass reduction, ~3× shorter arm. Frame has 4 motor mounts; 2 installed, 2 more planned.

First post-rebuild test run (2026-04-03): MAE=30.09°, Pearson r=−0.87 (inverted tracking). Root cause: IMU axis reorientation with new frame. AXIS_CENTER recalibrated to 275.

**Depends on:** hardware fabrication.

### Post-rebuild PID retuning — DONE (2026-04-07)

Full retuning required after mechanical rebuild invalidated all previous gains. Key finding: the controller must satisfy a **force budget** — `angle_kp × start_angle_error ≥ ANGLE_RATE_LIMIT` — or the outer loop never saturates the rate limit and produces insufficient differential thrust to lift from the restrictor. The frame is precisely balanced; the ~18 g of resistance at −59° is wire tension (cables routed outside the rotation axis, no slip ring) plus bearing friction — a roughly constant force, not gravity-dependent. Thrust slope: ~0.147 g/DShot unit at BASE=600.

Empirical thrust data (BetaFPV Lava 1104 7200KV, single motor):

| DShot value | Throttle % | Thrust |
|---|---|---|
| 200 | ~24% | 11 g |
| 500 | ~30% | 31 g |
| 600 | ~35% | 45 g |
| 800 | ~47% | 75 g |

Systematic tuning steps: angle_kp 1.0→3.5, angle_kd 0.1→0.3 (reduced oscillation 0.27→0.05 Hz), iterm_limit 30→100 (max I contribution 1.5→5 deg/s). BASE=500 tested and rejected — thrust curve asymmetry below BASE=600 reduces available differential from 19.5 g to 13.5 g.

**New baseline (run `2026-04-07_16-19-21`, 141.8s):**

| Metric | Value |
|---|---|
| Time to reach horizontal | 1.3 s |
| HoldMAE | 6.90° |
| Time at horizontal | 124.8 s |
| Oscillation freq | 0.05 Hz |
| Gains | angle kp=3.5 ki=0.05 kd=0.3 iterm_limit=100; rate kp=0.5 kd=0.003 |

Additional findings: IMU tare quality with precision jig + bubble level is ~0.10° residual — not the limiting factor. GRV dynamic lag (~0.8°) is the sensor floor independent of tare quality. Power supply limit (30W) requires LiHV batteries at BASE≥600; ~25s per charge.

See `decision/DR-008-cascaded-pid.md` Amendment 2026-04-07.

**Depends on:** mechanical rebuild, M4.

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

### Thrust linearization / expo (planned)

Motor thrust ∝ RPM², so effective thrust is nonlinear with DShot throttle value. `LeverMixer` outputs throttle values directly, meaning plant gain varies with operating point. A Betaflight-style expo mapping (`f(x) = (1−e)x + e·x³`, normalised) applied to the PID output before the mixer would reduce near-setpoint sensitivity without reducing authority at large errors — directly addressing the slow hold oscillation (0.05 Hz) observed in the current baseline. Planned as next tuning step after baseline is confirmed stable.

### I-term relax for large disturbances

During large transients the angle PID I-term continues accumulating during nonlinear large-signal
operation. Betaflight's "I-term relax" freezes integration when angular rate exceeds a threshold,
preventing windup during disturbances. Current `iterm_limit=100` (max I contribution 5 deg/s) is
large enough that post-disturbance overshoot is possible; revisit if characterization reveals
systematic hunting after large perturbations.

### Telemetry not flushed on crash

When an unhandled exception exits the control loop, `disarm()` is never reached and `telemetry.end_session()` is not called. The SD file is left open/unflushed; the last buffered rows may be lost. Fix: call `end_session()` from the crash path in `main()`, guarded so it does not raise if telemetry was never initialised.

## Test Bench vs Real Drone

See [DR-001, "Test Bench vs Real Drone" section](decision/DR-001-pid-lever-stabilization.md) for a detailed comparison covering: single axis vs three axes, single PID vs cascaded PIDs, encoder vs IMU, and fixed pivot vs free flight.

## Project Structure

```
├── src/                 # Authored flight control source (deployed to Pico)
│   ├── main.py          # Entry point — runs on boot
│   ├── pid.py           # PID controller with anti-windup and term introspection
│   ├── mixer.py         # LeverMixer — differential thrust for 2-motor lever
│   ├── ui.py            # Operator interface — buttons + RGB LED
│   └── telemetry/
│       ├── recorder.py  # TelemetryRecorder, PrintSink, SdSink, read_rtc
│       └── sdcard.py    # SD card SPI driver (micropython-lib, with stop bit fix)
├── AS5600/              # Git submodule: github.com/c0ffee2code/AS5600
├── BNO085/              # Git submodule: github.com/c0ffee2code/BNO085
├── DShot/               # Git submodule: github.com/c0ffee2code/DShot
├── tools/
│   ├── plot.py          # Generate diagnostic plots (run first — always)
│   ├── kpi.py           # Pass/fail gate: time-to-reach, HoldMAE, time-at-horizontal
│   └── analyse_telemetry.py  # Deep-dive stats for passing runs (no plots)
├── test_runs/           # Copied run folders from SD card for offline analysis
├── decision/            # Architecture Decision Records
└── resources/           # Datasheets, protocol docs
```

## Deployment

Clone with submodules:
```bash
git clone --recurse-submodules <repo-url>
```

Upload to Pico root (flat structure — no subdirectories on Pico):
- `src/main.py`
- `src/pid.py`
- `src/mixer.py`
- `src/ui.py`
- `src/telemetry/recorder.py` (as `recorder.py`)
- `src/telemetry/sdcard.py` (as `sdcard.py`)
- `AS5600/driver/as5600.py`
- `BNO085/driver/bno08x.py` + `BNO085/driver/i2c.py`
- `DShot/driver/dshot_pio.py` + `DShot/driver/motor_throttle_group.py`

## License

See individual submodule repositories for their licenses.