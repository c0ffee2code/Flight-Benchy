class PID:
    """Discrete PID controller with anti-windup and term introspection."""

    def __init__(self, kp, ki, kd=0.0, integral_limit=200.0):
        """Configure gains and integral windup limit."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def compute(self, error, dt):
        """Return PID output for given error and timestep. Updates last_p/last_i/last_d."""
        self._integral += error * dt
        if self._integral > self.integral_limit:
            self._integral = self.integral_limit
        elif self._integral < -self.integral_limit:
            self._integral = -self.integral_limit

        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        self.last_p = self.kp * error
        self.last_i = self.ki * self._integral
        self.last_d = self.kd * derivative
        return self.last_p + self.last_i + self.last_d

    def reset(self):
        """Zero integrator and derivative state for a fresh control session."""
        self._integral = 0.0
        self._prev_error = 0.0
