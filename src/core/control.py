from math import degrees, atan2

from core.pid import PID
from core.mixer import LeverMixer


def assert_motor_range(base, output_limit, throttle_min, throttle_max):
    """Raise ValueError if base +/- output_limit falls outside [throttle_min, throttle_max].

    Called at ControlCore init time so config errors surface before any motor movement.
    """
    if base + output_limit > throttle_max:
        raise ValueError(
            "Config: base_throttle({}) + output_limit({}) > throttle_max({})".format(
                base, output_limit, throttle_max))
    if base - output_limit < throttle_min:
        raise ValueError(
            "Config: base_throttle({}) - output_limit({}) < throttle_min({})".format(
                base, output_limit, throttle_min))


class ControlCore:
    """Hardware-free cascaded PID control logic for the lever bench.

    Owns all sign conversions, loop scheduling, and motor mixing.
    Call step() once per inner-loop cycle with raw IMU inputs.
    Post-step state is readable via last_* attributes and the pid/mixer objects.
    """

    def __init__(self, cfg, *, gyro_sign=-1, ff_sign=1, err_sign=1, mixer_sign=1):
        """
        cfg : dict
            Full config.json contents.
        gyro_sign, ff_sign, err_sign, mixer_sign : int
            Sign convention overrides. Production code always uses the defaults.
            Pass non-default values only for mutation testing (see tests/test_sil.py).
        """
        vehicle   = cfg["vehicle"]
        rate_hz   = vehicle["loops"]["rate"]["frequency_hz"]
        angle_hz  = vehicle["loops"]["angle"]["frequency_hz"]
        apid_cfg  = vehicle["loops"]["angle"]["pid"]
        rpid_cfg  = vehicle["loops"]["rate"]["pid"]
        motor_cfg = vehicle["motor"]
        ff_cfg    = vehicle["feedforward"]
        orient    = cfg["bench"]["sensor_orientation"]

        self._outer_ticks   = rate_hz // angle_hz
        self.outer_period_s = self._outer_ticks / rate_hz  # float division — QA-F2
        self._lead_s        = ff_cfg["lead_ms"] / 1000.0
        self._setpoint      = cfg["session"]["setpoint"]["roll_deg"]
        self._imu_sign      = -1 if orient["imu_invert"] else 1

        # Sign convention overrides (production: all defaults)
        self._gyro_sign  = gyro_sign
        self._ff_sign    = ff_sign
        self._err_sign   = err_sign
        self._mixer_sign = mixer_sign

        rate_output_limit = rpid_cfg.get("output_limit")
        if rate_output_limit is not None:
            assert_motor_range(
                motor_cfg["base_throttle"], rate_output_limit,
                motor_cfg["throttle_min"], motor_cfg["throttle_max"],
            )

        self.angle_pid = PID(
            kp=apid_cfg["kp"],
            ki=apid_cfg["ki"],
            kd=apid_cfg["kd"],
            iterm_limit=apid_cfg["iterm_limit"],
            output_limit=apid_cfg["output_limit"],
        )
        self.rate_pid = PID(
            kp=rpid_cfg["kp"],
            ki=rpid_cfg["ki"],
            kd=rpid_cfg["kd"],
            iterm_limit=rpid_cfg["iterm_limit"],
            output_limit=rpid_cfg["output_limit"],
        )
        self.mixer = LeverMixer(
            throttle_base=motor_cfg["base_throttle"],
            throttle_min=motor_cfg["throttle_min"],
            throttle_max=motor_cfg["throttle_max"],
        )

        # Loop state
        self._outer_counter = 0
        self._rate_setpoint = 0.0

        # Post-step telemetry state (updated before step() returns)
        self.last_gyro_x        = 0.0
        self.last_ang_err       = 0.0
        self.last_rate_setpoint = 0.0
        self.last_rate_error    = 0.0
        self.last_output        = 0.0
        self.last_is_outer      = False

    def step(self, iqr, iqi, iqj, iqk, raw_gx_rad, dt_s):
        """One control cycle. Returns (m1, m2, m3, m4) throttle commands.

        raw_gx_rad : gyroscope x reading from the IMU driver, in rad/s
        iqr/iqi/iqj/iqk : game rotation vector quaternion components
        dt_s : elapsed seconds since last call

        All last_* attributes are updated before returning.
        """
        gyro_x = self._gyro_sign * self._imu_sign * degrees(raw_gx_rad)

        self._outer_counter += 1
        is_outer = self._outer_counter >= self._outer_ticks
        if is_outer:
            self._outer_counter = 0
            imu_roll = self._imu_sign * degrees(2.0 * atan2(iqi, iqr))
            feedforward_roll = (imu_roll + self._ff_sign * gyro_x * self._lead_s) - self._setpoint
            ang_err = self._err_sign * feedforward_roll
            self.last_ang_err = ang_err
            self._rate_setpoint = self.angle_pid.compute(ang_err, self.outer_period_s)

        rate_error = self._rate_setpoint - gyro_x
        output = self.rate_pid.compute(rate_error, dt_s, measurement=gyro_x)
        m1, m2, m3, m4 = self.mixer.compute(self._mixer_sign * output)

        self.last_gyro_x        = gyro_x
        self.last_rate_setpoint = self._rate_setpoint
        self.last_rate_error    = rate_error
        self.last_output        = output
        self.last_is_outer      = is_outer

        return m1, m2, m3, m4

