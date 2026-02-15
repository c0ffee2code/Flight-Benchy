from micropython import const
from machine import I2C, Pin
from math import sin, cos, radians, degrees, atan2
import utime

from as5600 import AS5600, to_degrees
from bno08x import BNO08X
from i2c import BNO08X_I2C
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
imu = BNO08X_I2C(
    i2c,
    address=0x4A,
    reset_pin=Pin(PIN_IMU_RST, Pin.OUT),
    int_pin=Pin(PIN_IMU_INT, Pin.IN, Pin.PULL_UP),
)

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

# IMU report rate — 2x PID rate ensures fresh data each cycle
IMU_REPORT_HZ = const(100)

# Telemetry decimation: 1=every cycle, N=every Nth
TELEMETRY_SAMPLE_EVERY = const(10)

# Predictive correction — compensate IMU lag by extrapolating with angular rate
# ~60ms = BNO085 fusion filter group delay. See ADR-006 for latency budget.
LEAD_TIME_MS = const(60)

def set_led(r=0, g=0, b=0):
    """Set RGB LED state. Active LOW (common anode) — 0 turns LED on."""
    led_r.value(not r)
    led_g.value(not g)
    led_b.value(not b)


def angle_to_quat(deg):
    """Convert single-axis (roll/X) angle in degrees to quaternion (qr, qi, qj, qk)."""
    half = radians(deg) / 2
    return (cos(half), sin(half), 0.0, 0.0)


def buttons_by_held():
    """Return True if B+Y are both pressed (active low)."""
    return not btn_B.value() and not btn_Y.value()


# =====================================================
# State helpers
# =====================================================

def wait_for_arm():
    """Blue LED, block until B+Y held."""
    set_led(b=1)
    while not buttons_by_held():
        utime.sleep_ms(50)


def arm_motors(motors):
    """Green LED, enable IMU fusion, start DShot, arm ESCs."""
    set_led(g=1)
    imu.game_quaternion.enable(hertz=IMU_REPORT_HZ)
    motors.start()
    motors.arm()
    motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])


def wait_for_go():
    """Block until button A is pressed."""
    while btn_A.value():
        utime.sleep_ms(100)


def init_session(pid):
    """Create telemetry sink, write config, return TelemetryRecorder."""
    sink = SdSink(
        sck=PIN_SD_SCK, mosi=PIN_SD_MOSI,
        miso=PIN_SD_MISO, cs=PIN_SD_CS,
        rtc_sda=PIN_RTC_SDA, rtc_scl=PIN_RTC_SCL,
    )
    telemetry = TelemetryRecorder(TELEMETRY_SAMPLE_EVERY, sink=sink)
    config = (
        "# Flight Benchy run configuration\n"
        "imu:\n"
        "  report: game_rotation_vector\n"
        "  report_hz: {}\n"
        "pid:\n"
        "  hz: {}\n"
        "  kp: {}\n"
        "  ki: {}\n"
        "  kd: {}\n"
        "  integral_limit: {}\n"
        "motor:\n"
        "  base_throttle: {}\n"
        "  throttle_min: {}\n"
        "  throttle_max: {}\n"
        "encoder:\n"
        "  axis_center: {}\n"
        "telemetry:\n"
        "  sample_every: {}\n"
        "prediction:\n"
        "  lead_time_ms: {}\n"
    ).format(
        IMU_REPORT_HZ,
        1000 // PID_INTERVAL_MS,
        pid.kp, pid.ki, pid.kd, pid.integral_limit,
        BASE_THROTTLE, THROTTLE_MIN, THROTTLE_MAX,
        AXIS_CENTER,
        TELEMETRY_SAMPLE_EVERY,
        LEAD_TIME_MS,
    )
    telemetry.begin_session(config=config)
    return telemetry


def stabilize(pid, mixer, motors, telemetry):
    """Run PID control loop until B+Y disarm combo."""
    # Seed previous roll for first-cycle rate estimate
    imu.update_sensors()
    iqr, iqi, iqj, iqk = imu.game_quaternion
    prev_imu_roll = degrees(2.0 * atan2(iqi, iqr))
    prev_ms = utime.ticks_ms()

    lead_s = LEAD_TIME_MS / 1000.0

    while True:
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

        imu.update_sensors()
        iqr, iqi, iqj, iqk = imu.game_quaternion

        imu_roll = degrees(2.0 * atan2(iqi, iqr))

        # Predictive correction — estimate where lever is NOW
        angular_rate = (imu_roll - prev_imu_roll) / dt if dt > 0 else 0.0
        predicted_roll = imu_roll + angular_rate * lead_s
        prev_imu_roll = imu_roll

        enc_angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        eqr, eqi, eqj, eqk = angle_to_quat(enc_angle)

        output = pid.compute(predicted_roll, dt)
        m1, m2 = mixer.compute(output)

        motors.setThrottle(0, m1)
        motors.setThrottle(1, m2)

        telemetry.record(
            now_ms,
            eqr, eqi, eqj, eqk,
            iqr, iqi, iqj, iqk,
            predicted_roll, pid.last_p, pid.last_i, pid.last_d,
            output, m1, m2
        )


def disarm(motors, telemetry):
    """End telemetry session, disarm and stop motors, blue LED."""
    telemetry.end_session()
    motors.disarm()
    motors.stop()
    set_led(b=1)


# =====================================================
# Main
# =====================================================
def main():
    pid = PID(kp=3.5, ki=0.4, kd=0.3, integral_limit=50.0)
    mixer = LeverMixer(BASE_THROTTLE, THROTTLE_MIN, THROTTLE_MAX)
    motors = MotorThrottleGroup([Pin(PIN_MOTOR1), Pin(PIN_MOTOR2)], DSHOT_SPEEDS.DSHOT600)

    try:
        wait_for_arm()
        arm_motors(motors)
        wait_for_go()
        telemetry = init_session(pid)
        stabilize(pid, mixer, motors, telemetry)
        disarm(motors, telemetry)

    except Exception as e:
        set_led(r=1)
        raise

    finally:
        motors.stop()

main()
