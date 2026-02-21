# ADR-011: Power Telemetry — INA238 External Current/Voltage Monitor

**Status:** Accepted — hardware ordered, implementation pending
**Date:** 2026-02-21
**Context:** Add current and voltage monitoring to understand power draw, diagnose overcurrent events, and enable future throttle limiting

## Context

During M4 disturbance testing (2026-02-21), raising `THROTTLE_MAX` to 700 caused the bench PSU (12.2V supply) to trip its overcurrent protection at ~3.5A (~42.7W). The control loop has no awareness of power draw — it commands arbitrary motor differential within `THROTTLE_MAX` without knowing whether that saturates the ESC or overloads the supply.

Two requirements:

1. **Telemetry** — log instantaneous current and bus voltage alongside the existing PID/motor CSV columns, so post-run analysis can correlate throttle commands with actual power draw.
2. **Future throttle limiting** — provide a real-time current reading that can be fed back into the control loop as a soft ceiling (e.g. back off `THROTTLE_MAX` dynamically when current exceeds a threshold).

## Option Considered and Rejected: ESC On-Board Galvanometer

The JHEMCU Wing Dual 40A 2in1 ESC (BLHeli_S firmware) is described as having a "built-in ammeter function" accessible via a single telemetry wire. This was evaluated first as a zero-cost path.

### Findings

| Test | Reading | PSU current |
|------|---------|-------------|
| ESC powered, motors off | ~0V (−0.006V noise floor) | 0A |
| Motors spinning (via DShot, low throttle) | ~0V | >0A |
| Motors spinning (high throttle) | ~0V | >0A |
| Wire probed vs 5V rail | noise only | — |

Wire continuity confirmed good. Signal is absent under all load conditions.

### Why it was rejected

1. **Non-functional in practice** — three independent test conditions produced no signal. Most likely root cause: the sensor circuit requires VCC from the 8P FC connector socket, which was not connected. The protocol and pinout for that connector are not documented publicly; JHEMCU does not publish a schematic.
2. **Hardware-coupled** — if the ESC or PDB is replaced in future (e.g. upgrade to 4-in-1 ESC, brushless FC stack), the telemetry wire changes or disappears. Telemetry capability would be lost.
3. **BLHeli_S limitation** — BLHeli_S firmware does not support the standard ESC UART telemetry protocol (RPM/current/temperature) that BLHeli_32 provides. Any signal on the wire would be proprietary and undocumented.

## Decision

Add an **Adafruit INA238 breakout** ([product 6349](https://www.adafruit.com/product/6349)) in series on the main 12V power line between the bench PSU and the ESC.

### Why INA238

| Property | Value | Notes |
|----------|-------|-------|
| IC | INA238 | Texas Instruments 16-bit power monitor |
| Voltage range | 0–85V | Covers 12V bench with wide headroom |
| Shunt | 15mΩ, 0.1% tolerance | 52.5mV drop at 3.5A; 0.18W dissipation |
| Current resolution | 0.15mA/LSB (10A mode) | Sufficient for motor current profiling |
| Interface | I2C | Shares existing bus on GPIO 0/1 |
| Default I2C address | 0x40 | No conflict with AS5600/BNO085/PCF8523 |
| Logic level | 3.3V compatible | Direct Pico connection, no level shifter |
| Connector | Screw terminal (power) + STEMMA QT (I2C) | STEMMA QT compatible with Adalogger |
| Measures | Current + bus voltage + power | Single chip, single I2C read |

### Why not INA219

INA219 was the initial recommendation. INA238 is strictly better: 16-bit vs 12-bit ADC, lower shunt resistance (15mΩ vs 100mΩ → less wasted heat), higher voltage ceiling, and better accuracy (0.1% vs 0.5%). The driver complexity is similar.

### Why external sensor over on-board ESC telemetry

- **ESC-independent** — INA238 sits on the power line, not inside the ESC. Swapping ESC/PDB has no effect on current monitoring.
- **Measures total system current** — captures everything drawn from the supply (both motors + BEC + MCU), which is the value the PSU trips on.
- **Known interface** — standard I2C registers, TI datasheet, MicroPython driver straightforward to write.

## Wiring

```
PSU (+12V) ─── INA238 Vin+ ─── INA238 Vin- ─── ESC power (+)
PSU (GND)  ─────────────────────────────────── ESC power (−)

INA238 SDA ─── GPIO 0   (I2C bus 0, shared with AS5600 + BNO085)
INA238 SCL ─── GPIO 1
INA238 VCC ─── 3V3
INA238 GND ─── GND
```

The STEMMA QT connector on the INA238 breakout is compatible with the Adalogger PiCowbell's STEMMA QT port, making the I2C connection zero-solder if a STEMMA QT cable is available.

### Updated I2C bus topology

```
I2C Bus 0 (GPIO 0/1, 400 kHz)
├── AS5600 encoder     [0x36]
├── BNO085 IMU         [0x4A]
└── INA238 power mon.  [0x40]   ← new

SoftI2C (GPIO 4/5, 100 kHz)
└── PCF8523 RTC        [0x68]
```

## Implementation Plan (pending hardware)

1. **Driver** — minimal MicroPython I2C driver for INA238: configure shunt value, set averaging and conversion time, read current/voltage/power registers. No external library dependency.
2. **Telemetry** — add `AMPS` and `VOLTS` columns to the CSV schema (expanding from 22 to 24 columns). Update `TelemetryRecorder.record()` signature and `analyse_telemetry.py`.
3. **Soft throttle limiter** (future) — if `amps > CURRENT_LIMIT`, scale down `THROTTLE_MAX` in the mixer. Implemented after telemetry baseline is established.

## Consequences

### Positive

- Power draw visible in every telemetry run — correlate motor commands with actual current/voltage
- PSU trips become diagnosable rather than mysterious
- Independent of ESC firmware, vendor, or hardware generation
- Adds two new columns to CSV for free (no computational overhead in control loop — just an I2C read)
- INA238 also reports bus voltage — useful for detecting battery sag under load on a future battery-powered bench

### Negative / Trade-offs

- Adds one I2C transaction per control cycle (if sampled at inner loop rate). At 400 kHz with ~3 bytes, this is <0.1ms — negligible.
- Screw terminal must handle continuous motor current (up to 40A rated on ESC, though bench draws <4A). Standard breakout screw terminal ratings typically cover this at bench currents.
- CSV schema changes from 22 to 24 columns — old run folders are superseded. This is acceptable; the bench re-establishes baseline after each hardware or sensor change.

## Dependencies

- INA238 breakout (Adafruit 6349) — hardware ordered
- `main.py` — new I2C init + INA238 read in hot loop
- `telemetry/recorder.py` — extend `record()` to accept amps + volts
- `tools/analyse_telemetry.py` — new columns in stats and plots
