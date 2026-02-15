# ADR-004: Operator Interface — Buttons + RGB LED

**Status:** Accepted
**Date:** 2026-02-15
**Context:** Resolve SPI0 conflict between Adalogger SD card and Pimoroni Display Pack LCD

## Context

The Pimoroni Pico Display Pack and Adafruit PiCowbell Adalogger both claim SPI0 on GPIO 16–19 with incompatible pin roles (see ADR-002, "Pin Conflicts" section). They cannot coexist — when `SdSink` initializes SPI0 for SD card access, it reconfigures the bus the display depends on, freezing the LCD.

The display LCD was used for real-time status during development, but telemetry logging (M2a) and BNO085 high-speed SPI (future) need those SPI pins more. The LCD also added ~1145ms of overhead to the PID loop when `draw_stabilizing()` was called, far exceeding the 20ms target cycle time.

The display pack's **buttons** (GPIO 12–15) and **RGB LED** (GPIO 6/7/8) are simple digital I/O — they don't use SPI and have no bus conflicts.

## Decision

### Flash standard MicroPython

Replace Pimoroni custom firmware with standard MicroPython on Pico 2. PicoGraphics library is no longer needed since the LCD is disconnected.

### Rewire display pack

Connect the display pack to the Pico via breadboard with only:
- **4 buttons** on GPIO 12–15 (active LOW, directly connected)
- **RGB LED** on GPIO 6/7/8

LCD SPI pins left disconnected. This eliminates the SPI0 conflict entirely.

### Move motors from GPIO 6/7 to GPIO 10/11

Frees GPIO 6/7/8 for the RGB LED. DShot uses PIO state machines, which can drive any GPIO pin — the reassignment is purely software.

### LED color coding

| Color | GPIO | State | Meaning |
|-------|------|-------|---------|
| Blue | GP8 | DISARMED | Idle, safe to approach |
| Green | GP7 | ARMING / READY / STABILIZING | Motors active |
| Red | GP6 | ERROR | Exception, motors stopped |

Only one color active at a time. The LED is **active LOW** (common anode) — `set_led()` inverts the values so callers use intuitive `1=on` semantics.

### State machine

The multi-button UX is unchanged (B+Y to arm, A to start, B+Y to disarm). LCD `draw_*()` calls are replaced with LED color changes:

| State | LED | Enter trigger | Exit trigger |
|-------|-----|---------------|--------------|
| DISARMED | Blue | Boot / disarm | Hold B+Y |
| ARMING | Green | B+Y held | Auto (arm complete) |
| READY | Green | Arm complete | Press A |
| STABILIZING | Green | Press A | Hold B+Y |
| ERROR | Red | Exception | Restart |

### Resulting pin assignments

| GPIO | Constant | Function |
|------|----------|----------|
| 0 | `PIN_I2C0_SDA` | I2C bus 0 SDA — sensors |
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
| 16 | `PIN_SD_MISO` | Adalogger SD SPI0 MISO |
| 17 | `PIN_SD_CS` | Adalogger SD SPI0 CS |
| 18 | `PIN_SD_SCK` | Adalogger SD SPI0 SCK |
| 19 | `PIN_SD_MOSI` | Adalogger SD SPI0 MOSI |

## Consequences

### Positive

- **SPI0 conflict resolved** — SD card has exclusive use of SPI0; no bus contention
- **PID loop faster** — removing `draw_stabilizing()` eliminates ~1s of display SPI overhead per call from the control loop
- **Simpler firmware** — standard MicroPython, no Pimoroni custom build dependency
- **LED status visible from distance** — easier to monitor than reading an LCD
- **Buttons preserved** — same operator UX for arming/disarming

### Negative

- **No real-time data on display** — angle and throttle values no longer shown live during stabilization. Mitigated by telemetry logging to SD card (M2a) and serial REPL output.
- **LED polarity** — display pack RGB LED is active LOW (common anode). `set_led()` inverts values so the rest of the code uses intuitive `1=on` semantics.

### Files changed

- `main.py` — removed `display_pack` import and all `draw_*()` calls; motor pins 6/7 → 10/11; added LED pins 6/7/8 + `set_led()` helper; removed `DISPLAY_EVERY` constant and loop_count display logic
- `CLAUDE.md` — updated pin table, hardware list, deployment list
- `README.md` — updated hardware description, deployment list
- `ADR-002` — marked SPI0 conflict as resolved, updated motor pin references

### Not changed

- `pimoroni/pico_display_pack.py` — remains in repo for reference, not deployed or imported
- `telemetry/recorder.py`, `mixer.py`, `pid.py` — unchanged (no display coupling)

## Dependencies

- Motor pin reassignment in `main.py` (GPIO 6/7 → GPIO 10/11) — done
- Standard MicroPython firmware on Pico 2 — manual step (flash before deploying)
- Display pack breadboard rewiring — manual step (buttons + LED only)
