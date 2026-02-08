from micropython import const
from machine import I2C, Pin
import utime

from as5600 import AS5600, to_degrees
from motor_throttle_group import MotorThrottleGroup
from dshot_pio import DSHOT_SPEEDS
from display_pack import (draw_disarmed, draw_arming, draw_ready,
                          draw_stabilizing, draw_error)
from pid import PID

# =====================================================
# Hardware
# =====================================================
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
encoder = AS5600(i2c=i2c)

MOTOR1_PIN = Pin(4)
MOTOR2_PIN = Pin(5)

# Buttons (active LOW)
btn_A = Pin(12, Pin.IN, Pin.PULL_UP)
btn_B = Pin(13, Pin.IN, Pin.PULL_UP)
btn_Y = Pin(15, Pin.IN, Pin.PULL_UP)

# =====================================================
# Constants
# =====================================================
# Encoder calibration — recapture after mechanical changes!
AXIS_CENTER = const(422)

# Motor limits
THROTTLE_MIN = const(70)
THROTTLE_MAX = const(600)
BASE_THROTTLE = const(300)

# Control loop timing
PID_INTERVAL_MS = const(20)  # 50 Hz

# Display update (every N-th PID cycle to avoid display overhead each loop)
DISPLAY_EVERY = const(5)  # 10 Hz display refresh


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def buttons_by_held():
    """Return True if B+Y are both pressed (active low)."""
    return not btn_B.value() and not btn_Y.value()


# =====================================================
# Main
# =====================================================
def main():
    pid = PID(kp=5.0, ki=0.5, kd=0.0, integral_limit=200.0)
    motors = MotorThrottleGroup([MOTOR1_PIN, MOTOR2_PIN], DSHOT_SPEEDS.DSHOT600)

    try:
        while True:
            # ----- STATE 1: DISARMED -----
            draw_disarmed()
            while not buttons_by_held():
                utime.sleep_ms(50)

            # ----- STATE 2: ARMING -----
            motors.start()
            draw_arming()
            motors.arm()
            motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])

            # ----- STATE 3: READY CHECK -----
            while btn_A.value():  # wait for A press
                angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
                draw_ready(angle)
                utime.sleep_ms(100)

            # ----- STATE 4: STABILIZING -----
            pid.reset()
            loop_count = 0
            prev_ms = utime.ticks_ms()

            while True:
                # Check disarm combo
                if buttons_by_held():
                    break

                now_ms = utime.ticks_ms()
                dt_ms = utime.ticks_diff(now_ms, prev_ms)
                if dt_ms < PID_INTERVAL_MS:
                    utime.sleep_ms(PID_INTERVAL_MS - dt_ms)
                    now_ms = utime.ticks_ms()
                    dt_ms = utime.ticks_diff(now_ms, prev_ms)
                prev_ms = now_ms

                dt = dt_ms / 1000.0

                # Read angle
                angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)

                # PID — target is 0°, error = angle (sign verified on hardware)
                output = pid.compute(angle, dt)

                m1 = clamp(int(BASE_THROTTLE + output), THROTTLE_MIN, THROTTLE_MAX)
                m2 = clamp(int(BASE_THROTTLE - output), THROTTLE_MIN, THROTTLE_MAX)

                motors.setThrottle(0, m1)
                motors.setThrottle(1, m2)

                # Display at reduced rate
                loop_count += 1
                if loop_count >= DISPLAY_EVERY:
                    loop_count = 0
                    draw_stabilizing(angle, m1, m2)

            # ----- DISARM -----
            motors.disarm()
            motors.stop()

    except Exception as e:
        draw_error(str(e))
        raise

    finally:
        motors.stop()


main()
