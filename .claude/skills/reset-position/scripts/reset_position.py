"""
Open-loop position reset — drives M1 end to the restrictor (resting position).

Applies a fixed M1 throttle for RESET_DURATION_MS, then disarms.
Run via:  mpremote run .claude/skills/reset-position/scripts/reset_position.py
No SD card or encoder access needed — open-loop only.
"""
from micropython import const
from machine import Pin
import utime

from dshot_pio import DShotPIO, DSHOT_SPEEDS

PIN_MOTOR1 = const(10)

THROTTLE_MIN      = const(100)
THROTTLE_RESET    = const(350)   # above ~290 flip threshold (bench_sweep 2026-04-11)
RESET_DURATION_MS = const(3000)


def send_for_ms(motor, throttle, duration_ms):
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < duration_ms:
        motor.sendThrottleCommand(throttle)
        utime.sleep_us(1000)


def main():
    motor = DShotPIO(0, Pin(PIN_MOTOR1), DSHOT_SPEEDS.DSHOT600)
    motor.start()
    send_for_ms(motor, 0, 500)                   # arm ESC
    send_for_ms(motor, THROTTLE_MIN, 1500)        # idle before thrust
    print("Armed. Applying M1 reset thrust ({} for {}ms) ...".format(THROTTLE_RESET, RESET_DURATION_MS))
    send_for_ms(motor, THROTTLE_RESET, RESET_DURATION_MS)
    send_for_ms(motor, THROTTLE_MIN, 500)         # cooldown
    send_for_ms(motor, 0, 200)                    # disarm
    print("Done — M1 end at restrictor.")


main()
