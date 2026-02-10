class PrintSink:
    """Output backend that prints CSV rows to REPL serial console."""

    def write(self, line):
        """Emit a single CSV line to stdout."""
        print(line)

    def flush(self):
        """No-op — stdout is unbuffered."""
        pass


class TelemetryRecorder:
    """Facade that decimates and formats telemetry rows, delegating I/O to a sink."""

    _HEADER = "T_MS,ENC_DEG,IMU_DEG,ERR,P,I,D,PID_OUT,M1,M2"

    def __init__(self, sample_every, sink=None):
        """Set decimation rate and output backend (defaults to PrintSink)."""
        self._sample_every = sample_every
        self._sink = sink or PrintSink()
        self._counter = 0

    def begin_session(self):
        """Reset counter and emit CSV header. Call when entering STABILIZING state."""
        self._counter = 0
        self._sink.write(self._HEADER)

    def record(self, t_ms, enc_deg, imu_deg, err, p, i, d, pid_out, m1, m2):
        """Format and emit a CSV row every sample_every-th call. Others are silently dropped."""
        self._counter += 1
        if self._counter < self._sample_every:
            return
        self._counter = 0

        imu_s = "" if imu_deg is None else "{:.2f}".format(imu_deg)
        line = "{},{:.2f},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{}".format(
            t_ms, enc_deg, imu_s, err, p, i, d, pid_out, m1, m2
        )
        self._sink.write(line)

    def end_session(self):
        """Flush the sink. Call when leaving STABILIZING state (before disarm)."""
        self._sink.flush()
