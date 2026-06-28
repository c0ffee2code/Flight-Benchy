from micropython import const
from machine import I2C, Pin
import utime
import ujson
import gc

from as5600 import AS5600, to_degrees
from bno08x import BNO08X, SensorResetError
from i2c import BNO08X_I2C
from motor_throttle_group import MotorThrottleGroup
from dshot_pio import DSHOT_SPEEDS
from core.control import ControlCore
from recorder import TelemetryRecorder, SdSink
from pcf8523 import PCF8523
from ui import set_led

# =====================================================
# Pin assignments
# =====================================================
# I2C bus 0 — sensors (AS5600 encoder + BNO085 IMU + PCF8523 RTC)
PIN_I2C0_SDA   = const(0)
PIN_I2C0_SCL   = const(1)

# BNO085 IMU control
PIN_IMU_RST    = const(2)
PIN_IMU_INT    = const(3)

# DShot motors — M1/M2 moved from GP6/7 to free RGB LED pins (DR-004); M3/M4 on GP4/5
PIN_MOTOR1     = const(10)
PIN_MOTOR2     = const(11)
PIN_MOTOR3     = const(4)
PIN_MOTOR4     = const(5)

# SD card breakout — SPI0
PIN_SD_MISO    = const(16)
PIN_SD_CS      = const(17)
PIN_SD_SCK     = const(18)
PIN_SD_MOSI    = const(19)

# =====================================================
# Constants
# =====================================================
MS_PER_S          = const(1000)  # milliseconds per second — used for ms<->s conversions throughout

GC_INTERVAL_MS     = const(800)   # minimum gap between gc.collect() calls in the control loop
IMU_POLL_US        = const(500)   # yield when update_sensors() returns 0 (no new IMU data)
IMU_STALE_MS       = const(100)   # ms without any IMU packet before aborting
CTRL_DIR_TRIP_TICKS   = const(50)   # outer ticks saturated+diverging before abort (~0.5s at 100Hz)
SENSOR_COH_WARMUP_TICKS = const(200) # outer ticks before coherence fuse activates (~2s at 100Hz)
SENSOR_COH_THRESHOLD    = const(300) # coherence accumulator trip value (positive = wrong signs)

# =====================================================
# Hardware
# =====================================================
i2c = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
encoder = AS5600(i2c=i2c)

def load_config():
    with open("config.json") as f:
        return ujson.load(f)

# =====================================================
# Time conversion helpers
# =====================================================
def ms_to_s(ms):
    return ms / MS_PER_S

def s_to_ms(s):
    return int(s * MS_PER_S)

def hz_to_period_ms(hz):
    return MS_PER_S // hz

# =====================================================
# State helpers
# =====================================================

def enable_imu_reports(imu, grv_hz, imu_hz):
    """Enable GRV and calibrated gyro reports and activate the Motion Engine.

    Enables GRV (outer angle loop) and calibrated gyro (inner rate loop) at their
    respective rates, then calls begin_calibration() to activate ME for all three
    sensors (accel, gyro, mag) and load saved DCD from flash.

    Without begin_calibration(), ME stays inactive after every power-on regardless
    of previously saved DCD, and accuracy=0 on all readings for the entire session.

    TODO: add an accuracy gate after this call -- poll imu.game_quaternion until
    accuracy >= 2 before proceeding to arm_motors(). ME converges within a few
    seconds when DCD is fresh and the sensor is held still, but there is currently
    no explicit wait. The 5s motor settle in arm_motors() provides incidental time
    but does not check accuracy.
    """
    imu.game_quaternion.enable(hertz=grv_hz)
    imu.gyro.enable(hertz=imu_hz)
    for attempt in range(3):
        try:
            imu.begin_calibration()
            return
        except RuntimeError:
            if attempt == 2:
                raise
            utime.sleep_ms(500)

def arm_motors(motors, throttle_min):
    """Green LED, start DShot, arm ESCs, then idle at throttle_min.

    The settle delay absorbs the 0.25-0.5s ESC start-time stagger so all motors
    reach steady idle before the ramp begins.
    """
    settle_ms = 5000
    set_led(g=1)
    motors.start()
    motors.arm()
    motors.setAllThrottles([throttle_min, throttle_min, throttle_min, throttle_min])
    utime.sleep_ms(settle_ms)

def prespin_motors(motors, throttle_min, base):
    """Ramp all motors from throttle_min to base in steps, then dwell.

    10-unit increments every 100 ms limit inrush current so the supply stays live
    through the ramp. 1 s dwell lets motors reach steady RPM before the control
    loop opens and applies differential thrust.
    """
    if base <= throttle_min:
        return
    step_ms       = 100   # delay between throttle increments
    dwell_ms      = 1000  # wait after reaching base before control loop opens
    throttle_step = 10    # throttle units added per ramp step
    for t in range(throttle_min, base + throttle_step, throttle_step):
        v = min(t, base)
        motors.setAllThrottles([v, v, v, v])
        utime.sleep_ms(step_ms)
    utime.sleep_ms(dwell_ms)

def init_session(cfg, sink, dt):
    """Open a recording session and return TelemetryRecorder."""
    sink.init_session(dt)
    telemetry = TelemetryRecorder(cfg["bench"]["telemetry"]["sample_every"], sink=sink)
    telemetry.begin_session()
    return telemetry

class FuseImuStaleness:
    """Raises RuntimeError if no new gyro data arrives within stale_ms milliseconds."""

    def __init__(self, stale_ms):
        self.stale_ms = stale_ms
        self.last_host_ts = utime.ticks_ms()

    def update(self, host_ts_ms):
        self.last_host_ts = host_ts_ms

    def check(self):
        if utime.ticks_diff(utime.ticks_ms(), self.last_host_ts) > self.stale_ms:
            raise RuntimeError("FuseImuStaleness: no packet for >{}ms".format(self.stale_ms))


class FuseControlDirection:
    """Raises RuntimeError if error diverges while angle PID output is saturated.

    Saturated output that fails to shrink the error means the control direction is wrong:
    inverted sign, disconnected motor, or severely detuned gains.
    """

    def __init__(self, output_limit, trip_ticks):
        self.output_limit = output_limit
        self.trip_ticks   = trip_ticks
        self.counter      = 0
        self.prev_err     = 0.0

    def update(self, rate_setpoint, ang_err_abs):
        if self.output_limit and abs(rate_setpoint) >= self.output_limit:
            if ang_err_abs >= self.prev_err:
                self.counter += 1
            else:
                self.counter = 0
            if self.counter >= self.trip_ticks:
                raise RuntimeError("FuseControlDirection: error diverging under saturated output")
        else:
            self.counter = 0
        self.prev_err = ang_err_abs


class FuseSensorCoherence:
    """Raises RuntimeError if gyro and encoder rate signs are incoherent.

    Under the correct sign contract gyro_x = -phi_dot, so gyro_x * delta_enc <= 0 always.
    A positive accumulator beyond threshold means one of the signs is flipped.
    """

    def __init__(self, warmup_ticks, threshold):
        self.warmup_ticks = warmup_ticks
        self.threshold    = threshold
        self.warmup       = 0
        self.acc_sum      = 0.0
        self.prev_enc     = 0.0

    def update(self, gyro_x, enc_angle):
        if self.warmup < self.warmup_ticks:
            self.warmup += 1
            self.prev_enc = enc_angle
            return
        self.acc_sum += gyro_x * (enc_angle - self.prev_enc)
        self.prev_enc = enc_angle
        if self.acc_sum > self.threshold:
            raise RuntimeError("FuseSensorCoherence: gyro/encoder sign mismatch (sum={:.0f})".format(self.acc_sum))


def stabilize(core, motors, telemetry, imu, cfg, duration_ms=None):
    """Run cascaded PID control loop until duration_ms elapses.

    Inner (rate) loop runs on sensor data delivery: each iteration calls update_sensors()
    first; if no new report is available the cycle is skipped (500us yield, continue).
    Outer (angle) loop runs every outer_ticks confirmed inner cycles at angle_loop_hz.
    duration_ms=None means run indefinitely — cut power to stop.
    """
    axis_center = cfg["bench"]["encoder"]["axis_center"]
    enc_sign    = -1 if cfg["bench"]["sensor_orientation"]["encoder_invert"] else 1

    imu.update_sensors()  # flush initial packets
    run_start_ms  = utime.ticks_ms()
    prev_ms       = run_start_ms
    last_gc_ms    = run_start_ms
    last_gyro_ts  = 0.0  # sentinel; first real gyro packet (sensor_ts_ms > 0) always looks new

    imu_stale  = FuseImuStaleness(IMU_STALE_MS)
    ctrl_dir   = FuseControlDirection(core.angle_pid.output_limit, CTRL_DIR_TRIP_TICKS)
    sensor_coh = FuseSensorCoherence(SENSOR_COH_WARMUP_TICKS, SENSOR_COH_THRESHOLD)

    while True:
        if duration_ms is not None and utime.ticks_diff(utime.ticks_ms(), run_start_ms) >= duration_ms:
            break

        if imu.update_sensors() == 0:
            imu_stale.check()
            utime.sleep_us(IMU_POLL_US)
            continue

        g = imu.gyro.get()
        if g.sensor_ts_ms == last_gyro_ts:
            imu_stale.check()  # packet arrived but no new gyro — still enforce staleness
            continue

        now_ms = utime.ticks_ms()
        imu_stale.update(g.host_ts_ms)
        last_gyro_ts = g.sensor_ts_ms
        dt_ms  = utime.ticks_diff(now_ms, prev_ms)
        prev_ms = now_ms

        gx = g.data[0]
        grv = imu.game_quaternion.get()
        iqr, iqi, iqj, iqk = grv.data
        grv_lag_ms  = imu.bno_start_diff(grv.host_ts_ms) - grv.sensor_ts_ms
        gyro_lag_ms = imu.bno_start_diff(g.host_ts_ms)   - g.sensor_ts_ms

        m1, m2, m3, m4 = core.step(iqr, iqi, iqj, iqk, gx, ms_to_s(dt_ms))
        motors.setAllThrottles([m1, m2, m3, m4])

        enc_angle = enc_sign * to_degrees(encoder.read_raw_angle(), axis_center)

        telemetry.record(
            now_ms, dt_ms,
            enc_angle,
            iqr, iqi, iqj, iqk, grv.accuracy, grv_lag_ms,
            core.last_gyro_x, g.accuracy, gyro_lag_ms,
            core.last_ang_err,
            core.angle_pid.last_p, core.angle_pid.last_i, core.angle_pid.last_d,
            core.last_rate_setpoint,
            core.last_rate_error, core.rate_pid.last_p, core.rate_pid.last_i, core.rate_pid.last_d,
            core.last_output, m1, m2, m3, m4,
        )

        if core.last_is_outer:
            if utime.ticks_diff(now_ms, last_gc_ms) >= GC_INTERVAL_MS:
                gc.collect()
                last_gc_ms = now_ms

            ctrl_dir.update(core.last_rate_setpoint, abs(core.last_ang_err))
            sensor_coh.update(core.last_gyro_x, enc_angle)

# =====================================================
# Entry point
# =====================================================
def run():
    telemetry = None
    motors = None
    sd_sink = None
    try:
        cfg = load_config()

        rtc = PCF8523(i2c)
        sd_sink = SdSink(
            sck=PIN_SD_SCK,
            mosi=PIN_SD_MOSI,
            miso=PIN_SD_MISO,
            cs=PIN_SD_CS,
            preallocate_bytes=cfg["bench"]["telemetry"]["preallocate_bytes"],
        )
        telemetry = init_session(cfg, sd_sink, rtc.datetime())

        core = ControlCore(cfg)
        motors = MotorThrottleGroup(
            [Pin(PIN_MOTOR1), Pin(PIN_MOTOR2), Pin(PIN_MOTOR3), Pin(PIN_MOTOR4)],
            DSHOT_SPEEDS.DSHOT600,
        )

        imu = BNO08X_I2C(
            i2c,
            address=0x4A,
            reset_pin=Pin(PIN_IMU_RST, Pin.OUT),
            int_pin=Pin(PIN_IMU_INT, Pin.IN, Pin.PULL_UP),
        )
        enable_imu_reports(imu, cfg["vehicle"]["loops"]["angle"]["frequency_hz"], cfg["vehicle"]["loops"]["rate"]["frequency_hz"])
        arm_motors(motors, cfg["vehicle"]["motor"]["throttle_min"])
        prespin_motors(motors, cfg["vehicle"]["motor"]["throttle_min"], cfg["vehicle"]["motor"]["base_throttle"])
        duration_s = cfg["session"]["duration_s"]

        stabilize(core,
                  motors,
                  telemetry,
                  imu,
                  cfg,
                  duration_ms=s_to_ms(duration_s) if duration_s is not None else None)

    except Exception as e:
        set_led(r=1)
        if telemetry is not None:
            telemetry.write_crash_log(e)
        raise

    finally:
        if motors is not None:
            try:
                motors.stop()
            except Exception:
                pass
        set_led(b=1)
        if telemetry is not None:
            telemetry.end_session()
        elif sd_sink is not None:
            sd_sink.close()

if __name__ == '__main__':
    import sys
    try:
        run()
    except Exception as e:
        sys.print_exception(e, sys.stderr)
        sys.exit(1)
