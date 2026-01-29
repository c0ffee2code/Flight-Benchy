# DShot Protocol Reference

Sources:
- https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- https://www.betaflight.com/docs/development/API/Dshot

## Packet Structure

DShot frames are **16 bits** organized as:
```
SSSSSSSSSSSTCCCC
```
- **S** (11 bits): Throttle value (0-2047)
- **T** (1 bit): Telemetry request flag
- **C** (4 bits): CRC checksum

## Throttle Range

| Value | Purpose |
|-------|---------|
| 0 | Disarmed / Motor stop |
| 1-47 | Special commands |
| 48-2047 | Throttle (2000 steps) |

## Timing Specifications

| Version | Bitrate | T1H (high=1) | T0H (high=0) | Bit Period | Frame Duration |
|---------|---------|--------------|--------------|------------|----------------|
| DSHOT150 | 150 kbit/s | 5.00 µs | 2.50 µs | 6.67 µs | 106.72 µs |
| DSHOT300 | 300 kbit/s | 2.50 µs | 1.25 µs | 3.33 µs | 53.28 µs |
| DSHOT600 | 600 kbit/s | 1.25 µs | 0.625 µs | 1.67 µs | 26.72 µs |
| DSHOT1200 | 1200 kbit/s | 0.625 µs | 0.313 µs | 0.83 µs | 13.28 µs |

**Note:** Bits are distinguished by high-time duration within a fixed bit period.

## CRC Calculation

**Standard DShot:**
```python
crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
```

**Bidirectional DShot (inverted):**
```python
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F
```

## Arming Sequence

ESCs require sending throttle=0 for ~300ms before accepting throttle commands. This is the "arming" phase.

## Special Commands (0-47)

Commands only execute when motors are stopped.

| Command | Name | Notes |
|---------|------|-------|
| 0 | MOTOR_STOP | Reserved |
| 1-5 | BEEP1-5 | Wait 260ms between commands |
| 6 | ESC_INFO | Wait 12ms minimum |
| 7 | SPIN_DIRECTION_1 | Requires 6 transmissions |
| 8 | SPIN_DIRECTION_2 | Requires 6 transmissions |
| 9 | 3D_MODE_OFF | Requires 6 transmissions |
| 10 | 3D_MODE_ON | Requires 6 transmissions |
| 12 | SAVE_SETTINGS | Requires 6x, wait 35ms |
| 13 | EDT_ENABLE | EDT firmware only, requires 6x |
| 14 | EDT_DISABLE | EDT firmware only, requires 6x |
| 20 | SPIN_DIRECTION_NORMAL | Requires 6 transmissions |
| 21 | SPIN_DIRECTION_REVERSED | Requires 6 transmissions |
| 22-29 | LED control | On/off for LEDs 0-3 |
| 32-35 | Signal-line telemetry | Enable/disable eRPM reporting |
| 42 | Request temperature | |
| 43 | Request voltage | |
| 44 | Request current | |
| 45 | Request consumption | |
| 46 | Request eRPM | |
| 47 | Request eRPM period | |

## Bidirectional DShot

Two-way communication on single wire. Only works with DSHOT300+.

**Key differences:**
- Signal inverted: 1=low, 0=high
- CRC inverted (see above)
- ~30µs switching delay between TX and RX
- Effective frame rate roughly halves

### eRPM Response Frame (16 bits)

```
eeemmmmmmmmmcccc
```
- **e** (3 bits): Exponent (left-shift count)
- **m** (9 bits): Period base value
- **c** (4 bits): CRC (uninverted)

### GCR Encoding

eRPM values use Group Code Recording:
1. 16-bit value → 20-bit GCR via nibble mapping
2. Run-length encoding applied
3. Transmitted at 5/4× base DShot bitrate

Decoding: `gcr = (value ^ (value >> 1))`

## Extended DShot Telemetry (EDT)

Additional telemetry within eRPM frames (no extra wiring).

Frames starting with `0mmmmmmmm` (second bit = 0) indicate EDT:

| Type | Data |
|------|------|
| 0x02 | Temperature (°C) |
| 0x04 | Voltage (0.25V/step) |
| 0x06 | Current (amperes) |
| 0x08-0x0E | Debug values |

Requires firmware support on both ESC and flight controller.

## Hardware Compatibility

- **BLHeli_S**: DSHOT150/300 on EFM8BB1; all versions on newer MCUs
- **BLHeli_32**: All DSHOT versions
- **KISS ESCs**: All DSHOT versions
- **Bluejay firmware**: DSHOT only (no analog)
- **AM32 firmware**: Supports EDT

ESCs auto-detect protocol - no configuration needed.

## Frame Rate

Maximum theoretical: `1,000,000 / frame_duration`

| Version | Max Frames/sec |
|---------|----------------|
| DSHOT150 | ~9,370 |
| DSHOT300 | ~18,768 |
| DSHOT600 | ~37,425 |
| DSHOT1200 | ~74,850 |

For 32kHz PID loops, DSHOT600+ required to avoid bottleneck.

## Implementation Notes for Pico

Current implementation (`dshot/jrddupont/dshot_pio.py`) uses PIO with 8 cycles per bit:
- Cycles 0-1: Output low, read next bit
- Cycles 2-4: Output high (always)
- Cycles 5-7: Output high (if 1) or low (if 0)

PIO frequency = bitrate × 8 cycles/bit:
- DSHOT150: 1.2 MHz
- DSHOT300: 2.4 MHz
- DSHOT600: 4.8 MHz
- DSHOT1200: 9.6 MHz

## Project-Specific Goals

**Bidirectional DShot for this test bench:**

The test bench has multiple lag sources that affect control system performance:
1. **IMU lag** - delay between physical movement and sensor reading (measured via AS5600 reference)
2. **Motor response lag** - delay between throttle command and motor reaching target RPM

Implementing bidirectional DShot would enable:
- Real-time RPM feedback from ESCs
- Measuring motor response time (throttle → actual RPM)
- ESC health monitoring (temperature, voltage)
- Correlating commanded thrust with actual motor state

This data combined with IMU lag characterization will inform control loop design - the controller must predict not just where the lever *will be* (IMU lag) but also account for how long motors take to deliver requested thrust (motor lag).
