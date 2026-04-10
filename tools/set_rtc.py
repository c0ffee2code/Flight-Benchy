"""Utility: set PCF8523 RTC to a given timestamp.

Upload to Pico, edit the timestamp constants below, then run once.
The clock keeps time from the coin cell after power is removed.
"""
from machine import I2C, Pin

PIN_I2C0_SDA = const(0)
PIN_I2C0_SCL = const(1)
PCF8523_ADDR  = const(0x68)

# ── Edit before running ───────────────────────────────────────────
YEAR   = 2026
MONTH  = 4
DAY    = 10
HOUR   = 12
MINUTE = 1
SECOND = 0
# ─────────────────────────────────────────────────────────────────


def _bcd_enc(val):
    return ((val // 10) << 4) | (val % 10)


def _bcd_dec(val):
    return ((val >> 4) * 10) + (val & 0x0F)


def set_rtc(i2c, year, month, day, hour, minute, second):
    # Freeze oscillator (STOP=1, CAP_SEL=1 for 12.5 pF crystal on Adafruit breakout)
    i2c.writeto_mem(PCF8523_ADDR, 0x00, bytes([0xA0]))
    # Enable battery switch-over in standard mode, battery low detection on (PM[2:0]=000)
    i2c.writeto_mem(PCF8523_ADDR, 0x02, bytes([0x00]))
    # Write seconds–years (registers 0x03–0x09)
    i2c.writeto_mem(PCF8523_ADDR, 0x03, bytes([
        _bcd_enc(second),   # bit 7 (OS flag) = 0: clock integrity guaranteed
        _bcd_enc(minute),
        _bcd_enc(hour),
        _bcd_enc(day),
        0x00,               # weekday — not used by recorder.py, leave as 0
        _bcd_enc(month),
        _bcd_enc(year % 100),
    ]))
    # Restart oscillator (STOP=0, CAP_SEL=1)
    i2c.writeto_mem(PCF8523_ADDR, 0x00, bytes([0x80]))


def read_rtc(i2c):
    data = i2c.readfrom_mem(PCF8523_ADDR, 0x03, 7)
    if data[0] & 0x80:
        raise OSError("PCF8523: OS flag set — clock integrity not guaranteed, re-run set_rtc.py")
    return (
        2000 + _bcd_dec(data[6]),   # year
        _bcd_dec(data[5] & 0x1F),   # month
        _bcd_dec(data[3] & 0x3F),   # day
        _bcd_dec(data[2] & 0x3F),   # hour
        _bcd_dec(data[1] & 0x7F),   # minute
        _bcd_dec(data[0] & 0x7F),   # second
    )


i2c = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)

set_rtc(i2c, YEAR, MONTH, DAY, HOUR, MINUTE, SECOND)

yr, mon, day, hr, mn, sc = read_rtc(i2c)
print("RTC set to: {:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
    yr, mon, day, hr, mn, sc))
