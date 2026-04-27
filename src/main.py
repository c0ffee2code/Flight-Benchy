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
from ui import set_led, buttons_by_held, wait_for_arm, wait_for_go
from crash_log import write_crash_log

# =====================================================
# Pin assignments
# =====================================================
# I2C bus 0 — sensors (AS5600 encoder + BNO085 IMU + PCF8523 RTC)
PIN_I2C0_SDA   = const(0)
PIN_I2C0_SCL   = const(1)

# BNO085 IMU control
PIN_IMU_RST    = const(2)
PIN_IMU_INT    = const(3)

# DShot motors (moved from GP6/7 to free RGB LED pins, see DR-004)
PIN_MOTOR1     = const(10)
PIN_MOTOR2     = const(11)

# SD card breakout — SPI0
PIN_SD_MISO    = const(16)
PIN_SD_CS      = const(17)
PIN_SD_SCK     = const(18)
PIN_SD_MOSI    = const(19)

# =====================================================
# Hardware
# =====================================================
i2c = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
encoder = AS5600(i2c=i2c)

# =====================================================
# Constants
# =====================================================
# Encoder calibration — recapture after mechanical changes!
AXIS_CENTER = const(2378)

# Motor limits
THROTTLE_MIN = const(100)
THROTTLE_MAX = const(800)
BASE_THROTTLE = const(600)
MIXER_EXPO = 0.0       # expo shaping (DR-012). Set to 0.0 to disable.

# Control loop timing
INNER_INTERVAL_MS = const(5)       # 200 Hz inner (rate) loop
OUTER_INTERVAL_TICKS = const(4)    # outer runs every 4th inner tick = 50 Hz
OUTER_INTERVAL_MS = const(20)      # = INNER_INTERVAL_MS * OUTER_INTERVAL_TICKS; fixed outer dt

# IMU report rates
IMU_REPORT_HZ  = const(200)   # Calibrated Gyro — matches inner loop rate
GRV_REPORT_HZ  = const(50)    # Game Rotation Vector — matches outer loop rate

# Telemetry decimation: 1=every cycle, N=every Nth
TELEMETRY_SAMPLE_EVERY = const(10)

# Angle rate limit — max deg/s the angle loop can demand from the rate loop.
# Caps recovery speed for large disturbances; no effect near horizontal.
# Must satisfy: ANGLE_RATE_LIMIT * rate_kp >= gravity_imbalance_g / (2 * thrust_per_unit_g)
# At BASE=600: slope ~0.147 g/unit → need ANGLE_RATE_LIMIT * 0.5 >= 61 → ANGLE_RATE_LIMIT >= 122.
# Set to 130 for 10% margin. Max mixer output: BASE ± (rate_kp * 130) = 600 ± 65 = [535, 665].
ANGLE_RATE_LIMIT = const(130)

# Rate output limit — max throttle differential the rate loop can apply to the mixer.
# Prevents outlet protection triggering when large rate errors occur during fast transients
# (e.g. lever swings opposite to correction: rate_error = rate_sp ± fast_gyro can be 200+ deg/s).
# Natural ceiling without hitting throttle limits: min(THROTTLE_MAX−BASE, BASE−THROTTLE_MIN) = 300.
RATE_OUTPUT_LIMIT = const(300)

# Feedforward — compensate GRV filter lag in outer angle loop.
# Uses live calibrated gyro rate to extrapolate current angle.
# Matches measured GRV group delay (~10 - 20 ms). See DR-006.
FEEDFORWARD_LEAD_MS = const(15)

def angle_to_quat(deg):
    """Convert single-axis (roll/X) angle in degrees to quaternion (qr, qi, qj, qk)."""
    half = radians(deg) / 2
    return (cos(half), sin(half), 0.0, 0.0)


# =====================================================
# State helpers
# =====================================================

def enable_imu_reports(imu):
    """Enable GRV and calibrated gyro reports at their respective loop rates."""
    imu.game_quaternion.enable(hertz=GRV_REPORT_HZ)
    imu.gyro.enable(hertz=IMU_REPORT_HZ)

def arm_motors(motors):
    """Green LED, start DShot, arm ESCs."""
    set_led(g=1)
    motors.start()
    motors.arm()
    motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])

def init_session(angle_pid, rate_pid, sink):
    """Open a recording session on a pre-mounted sink, write config, return TelemetryRecorder."""
    sink.init_session()
    telemetry = TelemetryRecorder(TELEMETRY_SAMPLE_EVERY, sink=sink)
    config = (
        "# Flight Benchy run configuration\n"
        "imu:\n"
        "  angle_report: game_rotation_vector\n"
        "  angle_report_hz: {}\n"
        "  rate_report: calibrated_gyroscope\n"
        "  rate_report_hz: {}\n"
        "angle_pid:\n"
        "  hz: {}\n"
        "  kp: {}\n"
        "  ki: {}\n"
        "  kd: {}\n"
        "  iterm_limit: {}\n"
        "  output_limit: {}\n"
        "rate_pid:\n"
        "  hz: {}\n"
        "  kp: {}\n"
        "  ki: {}\n"
        "  kd: {}\n"
        "  iterm_limit: {}\n"
        "  output_limit: {}\n"
        "motor:\n"
        "  base_throttle: {}\n"
        "  throttle_min: {}\n"
        "  throttle_max: {}\n"
        "  expo: {}\n"
        "encoder:\n"
        "  axis_center: {}\n"
        "telemetry:\n"
        "  sample_every: {}\n"
        "feedforward:\n"
        "  lead_ms: {}\n"
    ).format(
        GRV_REPORT_HZ,
        IMU_REPORT_HZ,
        1000 // (INNER_INTERVAL_MS * OUTER_INTERVAL_TICKS),
        angle_pid.kp,
        angle_pid.ki,
        angle_pid.kd,
        angle_pid.iterm_limit,
        angle_pid.output_limit,
        1000 // INNER_INTERVAL_MS,
        rate_pid.kp,
        rate_pid.ki,
        rate_pid.kd,
        rate_pid.iterm_limit,
        rate_pid.output_limit,
        BASE_THROTTLE,
        THROTTLE_MIN,
        THROTTLE_MAX,
        MIXER_EXPO,
        AXIS_CENTER,
        TELEMETRY_SAMPLE_EVERY,
        FEEDFORWARD_LEAD_MS,
    )
    telemetry.begin_session(config=config)
    return telemetry


def stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu):
    """Run cascaded PID control loop until B+Y disarm combo.

    Inner (rate) loop runs every cycle at ~200 Hz.
    Outer (angle) loop runs every OUTER_INTERVAL_TICKS cycles (~50 Hz).
    """
    # Seed initial quaternion so telemetry has valid values before first outer tick
    imu.update_sensors()
    iqr, iqi, iqj, iqk, _acc, _ts = imu.game_quaternion.full
    prev_ms = utime.ticks_ms()

    feedforward_lead_s = FEEDFORWARD_LEAD_MS / 1000.0
    outer_dt = OUTER_INTERVAL_MS / 1000.0   # fixed nominal dt — avoids I2C-jitter noise in D term

    outer_counter = 0
    rate_setpoint = 0.0
    ang_err = 0.0  # holds last outer-loop angle error for telemetry

    while True:
        if buttons_by_held():
            break

        # --- timing: wait for inner tick ---
        now_ms = utime.ticks_ms()
        dt_ms = utime.ticks_diff(now_ms, prev_ms)
        if dt_ms < INNER_INTERVAL_MS:
            utime.sleep_ms(INNER_INTERVAL_MS - dt_ms)
            now_ms = utime.ticks_ms()
            dt_ms = utime.ticks_diff(now_ms, prev_ms)
        prev_ms = now_ms

        dt = dt_ms / 1000.0

        imu.update_sensors()
        gx, _gy, _gz = imu.gyro
        gyro_x = -degrees(gx)

        # --- outer loop (every OUTER_INTERVAL_TICKS cycles) ---
        outer_counter += 1
        if outer_counter >= OUTER_INTERVAL_TICKS:
            outer_counter = 0

            iqr, iqi, iqj, iqk = imu.game_quaternion
            imu_roll = -degrees(2.0 * atan2(iqi, iqr))

            # Feedforward (DR-006) — compensate GRV filter lag with live gyro rate
            feedforward_roll = -(imu_roll + gyro_x * feedforward_lead_s)
            ang_err = feedforward_roll

            rate_setpoint = angle_pid.compute(feedforward_roll, outer_dt)

        # --- inner loop (every cycle) ---
        rate_error = rate_setpoint - gyro_x
        output = rate_pid.compute(rate_error, dt, measurement=gyro_x)
        m1, m2 = mixer.compute(output)

        motors.setThrottle(0, m1)
        motors.setThrottle(1, m2)

        # --- encoder (read at inner rate for telemetry) ---
        # Frame was rotated during rebuild, reversing magnet direction; negate to restore sign convention.
        enc_angle = -to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        eqr, eqi, eqj, eqk = angle_to_quat(enc_angle)

        telemetry.record(
            now_ms,
            eqr, eqi, eqj, eqk,
            iqr, iqi, iqj, iqk,
            gyro_x,
            ang_err,
            angle_pid.last_p, angle_pid.last_i, angle_pid.last_d,
            rate_setpoint,
            rate_error, rate_pid.last_p, rate_pid.last_i, rate_pid.last_d,
            output, m1, m2
        )


# =====================================================
# Main
# =====================================================
def main():
    # --- Force budget (2026-04-07) ---
    # Empirical thrust: BASE=600 -> 45g/motor, slope ~0.147 g/throttle_unit near BASE.
    # Gravity imbalance at -59 deg: ~18g (single motor at BASE~300 just overcomes it).
    # Need: angle_kp * 58deg >= ANGLE_RATE_LIMIT (130) to saturate at start position.
    # angle_kp=1.0: rate_sp=58 -> never hits limit -> diff=8.7g (INSUFFICIENT).
    # angle_kp=2.2: rate_sp=129 -> saturates at 130 -> PID_OUT=65 -> diff=130 -> ~19g (OK).
    # rate_kp=0.5 confirmed stable at BASE=600 with kd=0.003. kp=0.7 caused 5.88Hz oscillation.
    # rate_kd raised 0.003→0.006 to add damping near setpoint without affecting DC gain (DR-012).
    angle_pid = PID(kp=3.5,  ki=0.05, kd=0.3,  iterm_limit=100.0, output_limit=ANGLE_RATE_LIMIT)  # outputs deg/s
    rate_pid  = PID(kp=0.5,  ki=0.0,  kd=0.006, iterm_limit=50.0, output_limit=RATE_OUTPUT_LIMIT) # outputs mixer scalar
    mixer = LeverMixer(throttle_base=BASE_THROTTLE, throttle_min=THROTTLE_MIN, throttle_max=THROTTLE_MAX, expo=MIXER_EXPO)
    motors = MotorThrottleGroup([Pin(PIN_MOTOR1), Pin(PIN_MOTOR2)], DSHOT_SPEEDS.DSHOT600)
    telemetry = None

    try:
        sd_sink = SdSink(
            sck=PIN_SD_SCK, mosi=PIN_SD_MOSI,
            miso=PIN_SD_MISO, cs=PIN_SD_CS,
            rtc_i2c=i2c,
        )
        imu = BNO08X_I2C(
            i2c,
            address=0x4A,
            reset_pin=Pin(PIN_IMU_RST, Pin.OUT),
            int_pin=Pin(PIN_IMU_INT, Pin.IN, Pin.PULL_UP),
        )
        wait_for_arm()
        enable_imu_reports(imu)
        arm_motors(motors)
        wait_for_go()
        telemetry = init_session(angle_pid, rate_pid, sd_sink)
        stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu)
        set_led(b=1)

    except Exception as e:
        set_led(r=1)
        raise

    finally:
        if telemetry is not None:
            telemetry.end_session()
        motors.disarm()
        motors.stop()

try:
    main()
except Exception as e:
    write_crash_log(e)
    raise
