def _bcd(b):
    return (b >> 4) * 10 + (b & 0x0F)


def read_rtc(i2c, addr=0x68):
    """Read current time from PCF8523 RTC.

    Returns (year, month, day, hour, minute, second).
    Raises OSError if the clock integrity flag is set (re-run set_rtc.py).
    """
    buf = i2c.readfrom_mem(addr, 0x03, 7)
    if buf[0] & 0x80:
        raise OSError("PCF8523: OS flag set — clock integrity not guaranteed, re-run set_rtc.py")
    return (
        _bcd(buf[6]) + 2000,   # year
        _bcd(buf[5] & 0x1F),   # month  (bits 7:5 reserved)
        _bcd(buf[3] & 0x3F),   # day    (bits 7:6 reserved)
        _bcd(buf[2] & 0x3F),   # hour   (bits 7:6 reserved)
        _bcd(buf[1] & 0x7F),   # minute (bit 7 reserved)
        _bcd(buf[0] & 0x7F),   # second (bit 7 = OS flag)
    )


class TimeSource:
    """Thin facade over the PCF8523 RTC. Returns wall-clock time on demand."""

    def __init__(self, i2c, addr=0x68):
        self._i2c = i2c
        self._addr = addr

    def now(self):
        """Return (year, month, day, hour, minute, second)."""
        return read_rtc(self._i2c, self._addr)