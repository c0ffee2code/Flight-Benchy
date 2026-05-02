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
from ui import set_led
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
# Constants
# =====================================================
PRESPIN_SETTLE_MS = const(1000)  # wait at throttle_min after arming — lets both ESCs finish their start sequence before the ramp begins
PRESPIN_STEP_MS   = const(50)   # delay between throttle increments during pre-spin ramp
PRESPIN_DWELL_MS  = const(500)  # wait after reaching base_throttle before starting the control loop

# =====================================================
# Hardware
# =====================================================
i2c     = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
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

def prespin_motors(motors, throttle_min, base):
    """Settle, ramp both motors from throttle_min to base, then dwell.

    PRESPIN_SETTLE_MS idles both motors at throttle_min first — observed 0.25–0.5s
    stagger between ESC start times means one motor can be pushing before the other
    has started; the settle delay lets both reach steady idle before the ramp begins.
    Ramping in 10-unit increments every PRESPIN_STEP_MS limits inrush current so the
    supply stays live through the ramp.
    PRESPIN_DWELL_MS dwell lets the motors reach steady RPM before the control loop
    opens and applies differential thrust.
    """
    utime.sleep_ms(PRESPIN_SETTLE_MS)
    for t in range(throttle_min, base + 10, 10):
        v = min(t, base)
        motors.setAllThrottles([v, v])
        utime.sleep_ms(PRESPIN_STEP_MS)
    utime.sleep_ms(PRESPIN_DWELL_MS)

def init_session(cfg, sink):
    """Open a recording session and return TelemetryRecorder."""
    sink.init_session()
    telemetry = TelemetryRecorder(cfg["telemetry"]["sample_every"], sink=sink)
    telemetry.begin_session()
    return telemetry


def stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu, cfg, duration_ms=None):
    """Run cascaded PID control loop until B+Y disarm combo or duration_ms elapses.

    Inner (rate) loop runs every cycle at ~200 Hz.
    Outer (angle) loop runs every outer_ticks cycles (~50 Hz).
    duration_ms=None means no time constraint — manual stop only.
    """
    inner_ms = 1000 // cfg["vehicle"]["imu"]["rate_report_hz"]
    outer_ticks = cfg["vehicle"]["imu"]["rate_report_hz"] // cfg["vehicle"]["imu"]["angle_report_hz"]
    outer_dt = (inner_ms * outer_ticks) / 1000.0  # fixed nominal dt — avoids I2C-jitter noise in D term
    axis_center = cfg["bench"]["encoder"]["axis_center"]
    feedforward_lead_s = cfg["vehicle"]["feedforward"]["lead_ms"] / 1000.0
    setpoint_roll_deg = cfg["bench"]["session"]["setpoint"]["roll_deg"]

    # Seed initial quaternion so telemetry has valid values before first outer tick
    imu.update_sensors()
    iqr, iqi, iqj, iqk, _acc, _ts = imu.game_quaternion.full
    run_start_ms = utime.ticks_ms()
    prev_ms = run_start_ms

    outer_counter = 0
    rate_setpoint = 0.0
    ang_err = 0.0  # holds last outer-loop angle error for telemetry

    while True:
        if duration_ms is not None and utime.ticks_diff(utime.ticks_ms(), run_start_ms) >= duration_ms:
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
            feedforward_roll = setpoint_roll_deg - (imu_roll + gyro_x * feedforward_lead_s)
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
# Entry point
# =====================================================
def run():
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
        kp=cfg["vehicle"]["angle_pid"]["kp"],
        ki=cfg["vehicle"]["angle_pid"]["ki"],
        kd=cfg["vehicle"]["angle_pid"]["kd"],
        iterm_limit=cfg["vehicle"]["angle_pid"]["iterm_limit"],
        output_limit=cfg["vehicle"]["angle_pid"]["output_limit"],
    )
    rate_pid = PID(
        kp=cfg["vehicle"]["rate_pid"]["kp"],
        ki=cfg["vehicle"]["rate_pid"]["ki"],
        kd=cfg["vehicle"]["rate_pid"]["kd"],
        iterm_limit=cfg["vehicle"]["rate_pid"]["iterm_limit"],
        output_limit=cfg["vehicle"]["rate_pid"]["output_limit"],
    )
    mixer = LeverMixer(
        throttle_base=cfg["vehicle"]["motor"]["base_throttle"],
        throttle_min=cfg["vehicle"]["motor"]["throttle_min"],
        throttle_max=cfg["vehicle"]["motor"]["throttle_max"],
        expo=cfg["vehicle"]["motor"]["expo"],
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
        enable_imu_reports(imu, cfg["vehicle"]["imu"]["angle_report_hz"], cfg["vehicle"]["imu"]["rate_report_hz"])
        arm_motors(motors, cfg["vehicle"]["motor"]["throttle_min"])
        prespin_motors(motors, cfg["vehicle"]["motor"]["throttle_min"], cfg["vehicle"]["motor"]["base_throttle"])
        telemetry = init_session(cfg, sd_sink)
        duration_s = cfg["bench"]["session"]["duration_s"]
        stabilize(angle_pid, rate_pid, mixer, motors, telemetry, imu, cfg,
                  duration_ms=int(duration_s * 1000) if duration_s is not None else None)
        set_led(b=1)

    except Exception:
        set_led(r=1)
        raise

    finally:
        if telemetry is not None:
            telemetry.end_session()
        motors.disarm()
        motors.stop()


if __name__ == '__main__':
    try:
        run()
    except Exception as e:
        write_crash_log(e)
        raise
