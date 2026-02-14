import os
import time
from machine import Pin, SPI, SoftI2C

import sdcard

_SD_MOUNT = "/sd"
_LOG_DIR  = _SD_MOUNT + "/blackbox"


def _bcd(b):
    """Decode a BCD byte to integer."""
    return (b >> 4) * 10 + (b & 0x0F)


def read_rtc(sda, scl, addr=0x68):
    """Read current time from RTC via one-shot SoftI2C.

    Returns (year, month, day, hour, minute, second).
    """
    i2c = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=100_000)
    buf = i2c.readfrom_mem(addr, 0x03, 7)
    return (
        _bcd(buf[6]) + 2000,  # year
        _bcd(buf[5] & 0x1F),  # month
        _bcd(buf[3]),          # day
        _bcd(buf[2]),          # hour
        _bcd(buf[1]),          # minute
        _bcd(buf[0] & 0x7F),  # second (bit 7 = OS flag)
    )


class PrintSink:
    """Output backend that prints CSV rows to REPL serial console."""

    def write(self, line):
        """Emit a single CSV line to stdout."""
        print(line)

    def flush(self):
        """No-op — stdout is unbuffered."""
        pass

    def close(self):
        """No-op."""
        pass


class SdSink:
    """Output backend that writes CSV rows to a file on a mounted SD card.

    Owns the full SD lifecycle: mount on init, unmount on close.
    The filename is generated from the RTC time at construction.
    Writes go directly to the file (no RAM buffering).
    """

    def __init__(self, sck, mosi, miso, cs, rtc_sda, rtc_scl):
        """Mount SD card, read RTC, and open a new log file.

        Args:
            sck, mosi, miso, cs: SD card SPI0 pin numbers.
            rtc_sda, rtc_scl: RTC SoftI2C pin numbers.
        """
        # Mount SD card
        cs_pin = Pin(cs, Pin.OUT, value=1)
        spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
                  sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        time.sleep_ms(250)
        self._sd = sdcard.SDCard(spi, cs_pin)
        self._vfs = os.VfsFat(self._sd)
        os.mount(self._vfs, _SD_MOUNT)

        # Read RTC for filename
        dt = read_rtc(sda=rtc_sda, scl=rtc_scl)

        # Create log directory and open file
        try:
            os.mkdir(_LOG_DIR)
        except OSError:
            pass  # already exists
        self._path = "{}/log_{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}.csv".format(
            _LOG_DIR, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
        )
        self._f = open(self._path, "w")

    @property
    def path(self):
        """Return the log file path (useful for diagnostics)."""
        return self._path

    def write(self, line):
        """Append a CSV line to the log file."""
        self._f.write(line)
        self._f.write("\n")

    def flush(self):
        """Flush buffered data to the SD card."""
        self._f.flush()

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        self._f.flush()
        self._f.close()
        os.umount(_SD_MOUNT)


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
        """Flush and close the sink. Call when leaving STABILIZING state (before disarm)."""
        self._sink.close()