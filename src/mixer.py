class LeverMixer:
    """Differential thrust mixer for a 4-motor lever (2 motors per end)."""

    def __init__(self, throttle_base, throttle_min, throttle_max):
        self.throttle_base = throttle_base
        self.throttle_min = throttle_min
        self.throttle_max = throttle_max

    def compute(self, pid_output):
        """Return (m1, m2, m3, m4) throttle values, clamped to limits.

        M3 is co-located with M1 (same lever end); M4 is co-located with M2.
        Both pairs receive identical signals.
        """
        m1 = self.throttle_base - int(pid_output)
        m2 = self.throttle_base + int(pid_output)

        m1 = max(self.throttle_min, min(self.throttle_max, m1))
        m2 = max(self.throttle_min, min(self.throttle_max, m2))
        return m1, m2, m1, m2
