class LeverMixer:
    """Differential thrust mixer for a 2-motor lever."""

    def __init__(self, throttle_base, throttle_min, throttle_max, expo=0.0):
        self.throttle_base = throttle_base
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max
        self.expo = expo
        self.authority_up   = float(throttle_max - throttle_base)   # max positive differential
        self.authority_down = float(throttle_base - throttle_min)   # max negative differential

    def linearise_thrust(self, pid_output):
        """Reduce near-setpoint differential authority to ease hold oscillation (DR-012)."""
        authority = self.authority_up if pid_output >= 0 else self.authority_down
        t = pid_output / authority
        if t < -1.0:
            t = -1.0
        elif t > 1.0:
            t = 1.0
        t = t * t * t * self.expo + t * (1.0 - self.expo)
        return t * authority

    def compute(self, pid_output):
        """Return (m1, m2) throttle values, clamped to limits."""
        if self.expo != 0.0:
            pid_output = self.linearise_thrust(pid_output)

        m1 = self.throttle_base - int(pid_output)
        m2 = self.throttle_base + int(pid_output)

        # clamp
        m1 = max(self.throttle_min, min(self.throttle_max, m1))
        m2 = max(self.throttle_min, min(self.throttle_max, m2))
        return m1, m2
