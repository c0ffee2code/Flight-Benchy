from dshot_pio import DShotPIO, DSHOT_SPEEDS
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


# =====================================================
# Buttons (active LOW)
# =====================================================
btn_A = Pin(12, Pin.IN, Pin.PULL_UP)  # M1 up
btn_B = Pin(13, Pin.IN, Pin.PULL_UP)  # M1 down / ARM
btn_X = Pin(14, Pin.IN, Pin.PULL_UP)  # M2 up
btn_Y = Pin(15, Pin.IN, Pin.PULL_UP)  # M2 down / ARM

# =====================================================
# Display setup (from pico pack wrapper)
# =====================================================
display = PicoGraphics(display=DISPLAY_PICO_DISPLAY, rotate=180)
display.set_backlight(1)

black = display.create_pen(0, 0, 0)
white = display.create_pen(255, 255, 255)

WIDTH, HEIGHT = display.get_bounds()

display.set_font("bitmap8")
SCALE = const(3)

X_COL_1 = const(0)
X_COL_2 = const(120)

Y_ROW_1 = const(0)
Y_ROW_2 = const(56)
Y_ROW_3 = const(111)

def draw_disarmed():
    display.set_pen(black)
    display.clear()
    display.set_pen(white)

    display.text("DISARMED", X_COL_1, Y_ROW_1, scale=SCALE)
    display.text("Hold B+Y", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("to ARM", X_COL_1, Y_ROW_3, scale=SCALE)

    display.update()

def draw_armed(th1, th2):
    display.set_pen(black)
    display.clear()
    display.set_pen(white)

    display.text("M1:", X_COL_1, Y_ROW_1, scale=SCALE)
    display.text(str(th1), X_COL_2, Y_ROW_1, scale=SCALE)

    display.text("M2:", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text(str(th2), X_COL_2, Y_ROW_2, scale=SCALE)

    display.text("ARMED", X_COL_1, Y_ROW_3, scale=SCALE)

    display.update()

# =====================================================
# Main
# =====================================================
def example():
    dshot_m1 = DShotPIO(0, 4, DSHOT_SPEEDS.DSHOT600)
    dshot_m2 = DShotPIO(1, 5, DSHOT_SPEEDS.DSHOT600)

    armed = False
    throttle_m1 = 0
    throttle_m2 = 0

    while True:
        # -------------------------
        # DISARMED STATE
        # -------------------------
        if not armed:
            draw_disarmed()

            # Always send zero throttle while disarmed
            dshot_m1.sendThrottleCommand(0)
            dshot_m2.sendThrottleCommand(0)

            # Check for ARM combo (B + Y held)
            if not btn_B.value() and not btn_Y.value():
                # Arm ESCs
                for _ in range(200):
                    dshot_m1.sendThrottleCommand(0)
                    dshot_m2.sendThrottleCommand(0)
                    utime.sleep_ms(10)

                throttle_m1 = THROTTLE_ARM
                throttle_m2 = THROTTLE_ARM
                armed = True

            utime.sleep_ms(50)
            continue

        # -------------------------
        # ARMED STATE
        # -------------------------
        if not btn_A.value():
            throttle_m1 += THROTTLE_STEP
        if not btn_B.value():
            throttle_m1 -= THROTTLE_STEP

        if not btn_X.value():
            throttle_m2 += THROTTLE_STEP
        if not btn_Y.value():
            throttle_m2 -= THROTTLE_STEP

        throttle_m1 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m1))
        throttle_m2 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m2))

        dshot_m1.sendThrottleCommand(throttle_m1)
        dshot_m2.sendThrottleCommand(throttle_m2)

        draw_armed(throttle_m1, throttle_m2)

        utime.sleep_ms(UPDATE_PERIOD_MS)

example()
