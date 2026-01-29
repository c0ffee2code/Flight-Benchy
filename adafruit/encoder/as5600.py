from machine import I2C, Pin
from micropython import const

# Encoder raw output is in steps
# Range is [0 ... 4096)
STEPS = const(4096)
DEG_PER_STEP = 360.0 / 4096.0   # ≈ 0.087890625°

#
DEFAULT_ADDR = const(0x36)
RAW_ANGLE_REG = const(0x0C)

# Sensor status registers
STATUS_REG = const(0x0B)
AGC_REG = const(0x1A)
MAGNITUDE_REG = const(0x1B)

def to_degrees(raw_angle, axis_center):
    steps_error = wrap_error(raw_angle - axis_center)
    return steps_error * DEG_PER_STEP

def wrap_error(err):
    """
    Normalize an angular error from a modulo encoder into the shortest signed distance.

    The AS5600 encoder reports angles modulo 4096 steps (0 and 4095 are adjacent).
    A naive subtraction of two angles can therefore produce large jumps near the
    wrap-around boundary (e.g. 4090 → 5 gives an error of -4085).

    This helper folds such values into the range [-2048, +2047], ensuring:
      - continuity across the 0/360° boundary
      - correct sign (direction of rotation)
      - a small, linear error suitable for control loops (PID)

    Input:
        err : int
            Raw difference between two encoder readings (e.g. current - reference)

    Returns:
        int
            Wrapped error representing the shortest angular distance in encoder steps.
    """
    if err > STEPS // 2:
        err -= STEPS
    elif err < -STEPS // 2:
        err += STEPS
    return err

# Minimalistic driver for AS5600 Encoder
class AS5600:

    def __init__(self, i2c, address=DEFAULT_ADDR, debug=False):
        self._i2c = i2c
        self._i2c_addr = address
        self._debug = debug

    def read_raw_angle(self):
        """ reads RAW Angle register and outputs magnet position. Returns value in the range [0, 4095]. """
        return self._read_12bit_register(RAW_ANGLE_REG)

    def status(self):
        """
            reads STATUS register and outputs status.
            returns: boolean magnet detected, boolean magnet too weak, magnet too strong
        """
        reg_status = self._read_8bit_register(STATUS_REG)
        reg_agc = self._read_agc()
        reg_magnitude = self._read_magnitude()

        status = {
            "magnet_detected": bool(reg_status & (1 << 5)),
            "magnet_too_weak": bool(reg_status & (1 << 4)),
            "magnet_too_strong": bool(reg_status & (1 << 3)),
            "agc": reg_agc,
            "magnitude": reg_magnitude,
        }

        self._dbg(status)
        return status

    def _read_magnitude(self):
        return self._read_12bit_register(MAGNITUDE_REG)

    def _read_agc(self):
        """
            AGC stands for automatic gain control,
            hich is used by the sensor for compensation of temperature and magnetic field variation.

            Adafruit encoder runs with 3.3v.

            Range is:
            with 5.5v -- [0, 255]
            with 3.3v -- [0, 128]
        """
        return self._read_8bit_register(AGC_REG)

    def _read_12bit_register(self, register):
        hi, lo = self._i2c.readfrom_mem(self._i2c_addr, register, 2)
        return ((hi & 0x0F) << 8) | lo

    def _read_8bit_register(self, register):
        return self._i2c.readfrom_mem(self._i2c_addr, register, 1)[0]

    def _dbg(self, *args, **kwargs) -> None:
        if self._debug:
            print("DBG:AS5600:\t", *args, **kwargs)