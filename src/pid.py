class PID:
    """Discrete PID controller with anti-windup and term introspection."""

    def __init__(self, kp, ki, kd=0.0, iterm_limit=200.0, output_limit=None):
        """Configure gains and integral windup limit."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.iterm_limit = iterm_limit
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def compute(self, error, dt, measurement=None):
        """Return PID output for given error and timestep. Updates last_p/last_i/last_d.

        measurement: if provided, D-term is computed from -d(measurement)/dt (measurement
        derivative) instead of d(error)/dt. This avoids derivative kick when the setpoint
        changes, which matters for the inner rate loop when rate_setpoint steps each outer tick.
        On the first call after reset, D is zero regardless.
        """
        self._integral += error * dt
        if self._integral > self.iterm_limit:
            self._integral = self.iterm_limit
        elif self._integral < -self.iterm_limit:
            self._integral = -self.iterm_limit

        if measurement is not None:
            if self._prev_measurement is None:
                derivative = 0.0
            else:
                derivative = -(measurement - self._prev_measurement) / dt if dt > 0 else 0.0
            self._prev_measurement = measurement
        else:
            derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        self.last_p = self.kp * error
        self.last_i = self.ki * self._integral
        self.last_d = self.kd * derivative
        output = self.last_p + self.last_i + self.last_d
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))
        return output

    def reset(self):
        """Zero integrator and derivative state for a fresh control session."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
