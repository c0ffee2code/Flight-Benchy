class LeverMixer:
    """Differential thrust mixer for a 2-motor lever."""

    def __init__(self, base, throttle_min, throttle_max):
        self.base = base
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max

    def compute(self, pid_output):
        """Return (m1, m2) throttle values, clamped to limits."""
        m1 = self.base + int(pid_output)
        m2 = self.base - int(pid_output)
        # clamp
        m1 = max(self.throttle_min, min(self.throttle_max, m1))
        m2 = max(self.throttle_min, min(self.throttle_max, m2))
        return m1, m2
