from dshot.jrddupont.dshot_pio import DShotPIO, DSHOT_SPEEDS
from machine import Pin
from picographics import PicoGraphics, DISPLAY_PICO_DISPLAY
from micropython import const
import utime

#
# D-Shot implementation and example:
# https://github.com/jrddupont/DShotPIO/blob/main/src/Example.py
#

# -----------------------------
# Configuration
#
# Throttle range [48 .. 2047]
# -----------------------------
THROTTLE_MIN = 65
THROTTLE_ARM = 65
THROTTLE_MAX = 600     # bench-safe limit
THROTTLE_STEP = 5
UPDATE_PERIOD_MS = 10

# -----------------------------
# Buttons (active LOW)
# -----------------------------
btn_A = Pin(12, Pin.IN, Pin.PULL_UP)  # M1 up
btn_B = Pin(13, Pin.IN, Pin.PULL_UP)  # M1 down
btn_X = Pin(14, Pin.IN, Pin.PULL_UP)  # M2 up
btn_Y = Pin(15, Pin.IN, Pin.PULL_UP)  # M2 down

# -----------------------------
# Main
# -----------------------------
def example():
    dshot_m1 = DShotPIO(
        stateMachineID=0,
        outputPin=4,
        dshotSpeed=DSHOT_SPEEDS.DSHOT600
    )
    dshot_m2 = DShotPIO(
        stateMachineID=1,
        outputPin=5,
        dshotSpeed=DSHOT_SPEEDS.DSHOT600
    )

    # -------------------------
    # Arm ESCs
    # -------------------------
    for _ in range(200):
        dshot_m1.sendThrottleCommand(0)
        dshot_m2.sendThrottleCommand(0)
        utime.sleep_ms(10)

    print("ARM Done")

    throttle_m1 = THROTTLE_ARM
    throttle_m2 = THROTTLE_ARM

    # -------------------------
    # Control loop
    # -------------------------
    while True:
        # ---- Motor 1 ----
        if not btn_A.value():  # A pressed
            throttle_m1 += THROTTLE_STEP
        if not btn_B.value():  # B pressed
            throttle_m1 -= THROTTLE_STEP

        # ---- Motor 2 ----
        if not btn_X.value():  # Y pressed
            throttle_m2 += THROTTLE_STEP
        if not btn_Y.value():  # X pressed
            throttle_m2 -= THROTTLE_STEP

        # Clamp throttles
        throttle_m1 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m1))
        throttle_m2 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m2))

        # Send commands
        dshot_m1.sendThrottleCommand(throttle_m1)
        dshot_m2.sendThrottleCommand(throttle_m2)

        utime.sleep_ms(UPDATE_PERIOD_MS)

example()
