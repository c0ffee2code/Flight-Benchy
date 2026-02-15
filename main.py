from micropython import const
from machine import I2C, Pin
import utime

from as5600 import AS5600, to_degrees
from motor_throttle_group import MotorThrottleGroup
from dshot_pio import DSHOT_SPEEDS
from pid import PID
from recorder import TelemetryRecorder, SdSink
from mixer import LeverMixer

# =====================================================
# Pin assignments
# =====================================================
# I2C bus 0 — sensors (AS5600 encoder + BNO085 IMU)
PIN_I2C0_SDA   = const(0)
PIN_I2C0_SCL   = const(1)

# BNO085 IMU control
PIN_IMU_RST    = const(2)
PIN_IMU_INT    = const(3)

# Adalogger RTC — SoftI2C (GP4/5 are I2C0 alt pins, bit-bang to avoid conflict)
PIN_RTC_SDA    = const(4)
PIN_RTC_SCL    = const(5)

# RGB LED from display pack (active HIGH, accent colors only — LCD disconnected)
PIN_LED_R      = const(6)   # Red — error
PIN_LED_G      = const(7)   # Green — armed / stabilizing
PIN_LED_B      = const(8)   # Blue — idle / ready to arm

# DShot motors (moved from GP6/7 to free RGB LED pins, see ADR-004)
PIN_MOTOR1     = const(10)
PIN_MOTOR2     = const(11)

# Display buttons (active LOW)
PIN_BTN_A      = const(12)
PIN_BTN_B      = const(13)
PIN_BTN_X      = const(14)
PIN_BTN_Y      = const(15)

# Adalogger SD card — SPI0
PIN_SD_MISO    = const(16)
PIN_SD_CS      = const(17)
PIN_SD_SCK     = const(18)
PIN_SD_MOSI    = const(19)

# =====================================================
# Hardware
# =====================================================
i2c = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
encoder = AS5600(i2c=i2c)

led_r = Pin(PIN_LED_R, Pin.OUT)
led_g = Pin(PIN_LED_G, Pin.OUT)
led_b = Pin(PIN_LED_B, Pin.OUT)

btn_A = Pin(PIN_BTN_A, Pin.IN, Pin.PULL_UP)
btn_B = Pin(PIN_BTN_B, Pin.IN, Pin.PULL_UP)
btn_Y = Pin(PIN_BTN_Y, Pin.IN, Pin.PULL_UP)

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

# Telemetry decimation: 1=every cycle, N=every Nth
TELEMETRY_SAMPLE_EVERY = const(10)

def set_led(r=0, g=0, b=0):
    """Set RGB LED state. Active LOW (common anode) — 0 turns LED on."""
    led_r.value(not r)
    led_g.value(not g)
    led_b.value(not b)


def buttons_by_held():
    """Return True if B+Y are both pressed (active low)."""
    return not btn_B.value() and not btn_Y.value()

# =====================================================
# Main
# =====================================================
def main():
    pid = PID(kp=5.0, ki=0.5, kd=0.0, integral_limit=200.0)
    mixer = LeverMixer(BASE_THROTTLE, THROTTLE_MIN, THROTTLE_MAX)
    motors = MotorThrottleGroup([Pin(PIN_MOTOR1), Pin(PIN_MOTOR2)], DSHOT_SPEEDS.DSHOT600)

    try:
        while True:
            # ----- STATE 1: DISARMED -----
            set_led(b=1)
            while not buttons_by_held():
                utime.sleep_ms(50)

            # ----- STATE 2: ARMING -----
            motors.start()
            set_led(g=1)
            motors.arm()
            motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])

            # ----- STATE 3: READY CHECK -----
            while btn_A.value():  # wait for A press
                utime.sleep_ms(100)

            # ----- STATE 4: STABILIZING -----
            pid.reset()
            sink = SdSink(
                sck=PIN_SD_SCK, mosi=PIN_SD_MOSI,
                miso=PIN_SD_MISO, cs=PIN_SD_CS,
                rtc_sda=PIN_RTC_SDA, rtc_scl=PIN_RTC_SCL,
            )
            telemetry = TelemetryRecorder(TELEMETRY_SAMPLE_EVERY, sink=sink)
            telemetry.begin_session()
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

                m1, m2 = mixer.compute(output)

                motors.setThrottle(0, m1)
                motors.setThrottle(1, m2)

                telemetry.record(
                    now_ms, angle, None,
                    angle, pid.last_p, pid.last_i, pid.last_d,
                    output, m1, m2
                )

            # ----- DISARM -----
            telemetry.end_session()
            motors.disarm()
            motors.stop()

    except Exception as e:
        set_led(r=1)
        raise

    finally:
        motors.stop()

main()
