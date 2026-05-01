from micropython import const
from machine import I2C, Pin
from math import sin, cos, radians, degrees, atan2
import utime
import ujson

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

def load_config():
    """Load config.json from Pico root. Red LED + raise if missing."""
    try:
        with open("config.json") as f:
            return ujson.load(f)
    except OSError:
        set_led(r=1)
        raise RuntimeError("config.json missing — upload via deploy skill")


def angle_to_quat(deg):
    """Convert single-axis (roll/X) angle in degrees to quaternion (qr, qi, qj, qk)."""
    half = radians(deg) / 2
    return (cos(half), sin(half), 0.0, 0.0)


# =====================================================
# State helpers
# =====================================================

def enable_imu_reports(imu, grv_hz, imu_hz):
    """Enable GRV and calibrated gyro reports at their respective loop rates."""
    imu.game_quaternion.enable(hertz=grv_hz)
    imu.gyro.enable(hertz=imu_hz)

def arm_motors(motors, throttle_min):
    """Green LED, start DShot, arm ESCs."""
    set_led(g=1)
    motors.start()
    motors.arm()
    motors.setAllThrottles([throttle_min, throttle_min])

def init_session(cfg, sink):
    """Open a recording session and return TelemetryRecorder."""
    sink.init_session()
    telemetry = TelemetryRecorder(cfg["telemetry"]["sample_every"], sink=sink)
    telemetry.begin_session()
    return telemetry


def stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu, cfg):
    """Run cascaded PID control loop until B+Y disarm combo.

    Inner (rate) loop runs every cycle at ~200 Hz.
    Outer (angle) loop runs every outer_ticks cycles (~50 Hz).
    """
    inner_ms = 1000 // cfg["imu"]["rate_report_hz"]
    outer_ticks = cfg["imu"]["rate_report_hz"] // cfg["imu"]["angle_report_hz"]
    outer_dt = (inner_ms * outer_ticks) / 1000.0  # fixed nominal dt — avoids I2C-jitter noise in D term
    axis_center = cfg["encoder"]["axis_center"]
    feedforward_lead_s = cfg["feedforward"]["lead_ms"] / 1000.0

    # Seed initial quaternion so telemetry has valid values before first outer tick
    imu.update_sensors()
    iqr, iqi, iqj, iqk, _acc, _ts = imu.game_quaternion.full
    prev_ms = utime.ticks_ms()

    outer_counter = 0
    rate_setpoint = 0.0
    ang_err = 0.0  # holds last outer-loop angle error for telemetry

    while True:
        if buttons_by_held():
            break

        # --- timing: wait for inner tick ---
        now_ms = utime.ticks_ms()
        dt_ms = utime.ticks_diff(now_ms, prev_ms)
        if dt_ms < inner_ms:
            utime.sleep_ms(inner_ms - dt_ms)
            now_ms = utime.ticks_ms()
            dt_ms = utime.ticks_diff(now_ms, prev_ms)
        prev_ms = now_ms

        dt = dt_ms / 1000.0

        imu.update_sensors()
        gx, _gy, _gz = imu.gyro
        gyro_x = -degrees(gx)

        # --- outer loop (every outer_ticks cycles) ---
        outer_counter += 1
        if outer_counter >= outer_ticks:
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
        enc_angle = -to_degrees(encoder.read_raw_angle(), axis_center)
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
    cfg = load_config()

    # --- Force budget (2026-04-07) ---
    # Empirical thrust: BASE=600 -> 45g/motor, slope ~0.147 g/throttle_unit near BASE.
    # Gravity imbalance at -59 deg: ~18g (single motor at BASE~300 just overcomes it).
    # Need: angle_kp * 58deg >= angle_pid.output_limit to saturate at start position.
    # angle_kp=1.0: rate_sp=58 -> never hits limit -> diff=8.7g (INSUFFICIENT).
    # angle_kp=2.2: rate_sp=129 -> saturates at 130 -> PID_OUT=65 -> diff=130 -> ~19g (OK).
    # rate_kp=0.5 confirmed stable at BASE=600 with kd=0.003. kp=0.7 caused 5.88Hz oscillation.
    # rate_kd raised 0.003->0.006 to add damping near setpoint without affecting DC gain (DR-012).
    angle_pid = PID(
        kp=cfg["angle_pid"]["kp"],
        ki=cfg["angle_pid"]["ki"],
        kd=cfg["angle_pid"]["kd"],
        iterm_limit=cfg["angle_pid"]["iterm_limit"],
        output_limit=cfg["angle_pid"]["output_limit"],
    )
    rate_pid = PID(
        kp=cfg["rate_pid"]["kp"],
        ki=cfg["rate_pid"]["ki"],
        kd=cfg["rate_pid"]["kd"],
        iterm_limit=cfg["rate_pid"]["iterm_limit"],
        output_limit=cfg["rate_pid"]["output_limit"],
    )
    mixer = LeverMixer(
        throttle_base=cfg["motor"]["base_throttle"],
        throttle_min=cfg["motor"]["throttle_min"],
        throttle_max=cfg["motor"]["throttle_max"],
        expo=cfg["motor"]["expo"],
    )
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
        enable_imu_reports(imu, cfg["imu"]["angle_report_hz"], cfg["imu"]["rate_report_hz"])
        arm_motors(motors, cfg["motor"]["throttle_min"])
        wait_for_go()
        telemetry = init_session(cfg, sd_sink)
        stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu, cfg)
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
