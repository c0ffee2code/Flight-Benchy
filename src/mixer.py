class LeverMixer:
    """Differential thrust mixer for a 2-motor lever."""

    def __init__(self, base, throttle_min, throttle_max, expo=0.0):
        self.base = base
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max
        self.expo = expo
        self.expo_limit = float(throttle_max - base)

    def compute(self, pid_output):
        """Return (m1, m2) throttle values, clamped to limits."""
        if self.expo > 0.0:
            t = pid_output / self.expo_limit
            if t < -1.0:
                t = -1.0
            elif t > 1.0:
                t = 1.0
            t = t * t * t * self.expo + t * (1.0 - self.expo)
            pid_output = t * self.expo_limit

        m1 = self.base - int(pid_output)
        m2 = self.base + int(pid_output)

        # clamp
        m1 = max(self.throttle_min, min(self.throttle_max, m1))
        m2 = max(self.throttle_min, min(self.throttle_max, m2))
        return m1, m2
