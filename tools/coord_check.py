# Coordinate system diagnostic — no motors.
# Establishes AXIS_CENTER and confirms encoder + IMU sign conventions
# after any mechanical reassembly.
#
# Upload to Pico and run once. Logs to /sd/coord_check/YYYY-MM-DD_hh-mm-ss/log.csv.
#
# Phases:
#   A — raw encoder readout until AXIS_CENTER captured (press A at horizontal)
#   B — dual readout until IMU tared (press B at horizontal)
#   C — live readout; mark extremes (X = M1-end down, Y = M2-end down)
#   D — same as C but at least one extreme has been marked
#
# Exit: hold A+B together (any time after Phase C starts).

from machine import I2C, Pin, SPI
from math import degrees, atan2
from micropython import const
import utime
import os
import time

from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C
import sdcard
from recorder import read_rtc
from ui import set_led, btn_A, btn_B, btn_X, btn_Y

# =====================================================
# Pin assignments
# =====================================================
PIN_I2C0_SDA = const(0)
PIN_I2C0_SCL = const(1)
PIN_IMU_RST  = const(2)
PIN_IMU_INT  = const(3)

PIN_SD_MISO  = const(16)
PIN_SD_CS    = const(17)
PIN_SD_SCK   = const(18)
PIN_SD_MOSI  = const(19)

IMU_REPORT_HZ = const(50)

_SD_MOUNT = "/sd"
_LOG_DIR  = _SD_MOUNT + "/coord_check"
_HEADER   = "T_MS,PHASE,ENC_RAW,ENC_DEG,IMU_ROLL,GYRO_X,MARKER"
_INTERVAL_MS = const(200)


# =====================================================
# Helpers
# =====================================================

def wait_release(pin):
    """Wait until button is released, then debounce."""
    while pin.value() == 0:
        utime.sleep_ms(10)
    utime.sleep_ms(30)


def read_sensors(imu, encoder, axis_center):
    """Return (t_ms, enc_raw, enc_deg, imu_roll, gyro_x)."""
    imu.update_sensors()
    qr, qi, qj, qk = imu.game_quaternion
    gx, _gy, _gz = imu.gyro
    enc_raw = encoder.read_raw_angle()
    return (
        utime.ticks_ms(),
        enc_raw,
        to_degrees(enc_raw, axis_center),
        degrees(2.0 * atan2(qi, qr)),
        degrees(gx),
    )


def log_row(f, t_ms, phase, enc_raw, enc_deg, imu_roll, gyro_x, marker=""):
    f.write("{},{},{},{:.2f},{:.2f},{:.2f},{}\n".format(
        t_ms, phase, enc_raw, enc_deg, imu_roll, gyro_x, marker
    ))


def mount_sd_and_open_log(i2c):
    cs_pin = Pin(PIN_SD_CS, Pin.OUT, value=1)
    spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
              sck=Pin(PIN_SD_SCK), mosi=Pin(PIN_SD_MOSI), miso=Pin(PIN_SD_MISO))
    time.sleep_ms(250)
    sd = sdcard.SDCard(spi, cs_pin)
    vfs = os.VfsFat(sd)
    os.mount(vfs, _SD_MOUNT)

    dt = read_rtc(i2c)
    try:
        os.mkdir(_LOG_DIR)
    except OSError:
        pass
    run_dir = "{}/{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
        _LOG_DIR, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
    )
    os.mkdir(run_dir)
    f = open(run_dir + "/log.csv", "w")
    f.write(_HEADER + "\n")
    return f, run_dir


# =====================================================
# Summary
# =====================================================

def _summary_lines(axis_center, m1_down, m2_down):
    """Return summary as a list of strings (shared by console and file output)."""
    lines = []
    lines.append("AXIS_CENTER (new): {}".format(axis_center))
    lines.append("")

    if not m1_down or not m2_down:
        lines.append("WARNING: one or both extremes not captured.")
        lines.append("M1-down: {}".format(
            "enc={:+.1f}  imu_roll={:+.1f}".format(*m1_down[:2]) if m1_down else "NOT CAPTURED"))
        lines.append("M2-down: {}".format(
            "enc={:+.1f}  imu_roll={:+.1f}".format(*m2_down[:2]) if m2_down else "NOT CAPTURED"))
        return lines

    m1_enc, m1_imu, _ = m1_down
    m2_enc, m2_imu, _ = m2_down

    lines.append("{:<12} {:>8}  {:>10}".format("Position", "ENC_DEG", "IMU_ROLL"))
    lines.append("{:<12} {:>+8.2f}  {:>+10.2f}".format("M1-down", m1_enc, m1_imu))
    lines.append("{:<12} {:>+8.2f}  {:>+10.2f}".format("M2-down", m2_enc, m2_imu))
    lines.append("")

    enc_ok = m1_enc < 0 < m2_enc
    imu_ok = m1_imu < 0 < m2_imu

    lines.append("Encoder:  M1-down reads {}  {}".format(
        "NEGATIVE" if m1_enc < 0 else "POSITIVE",
        "(matches old convention)" if enc_ok else "(INVERTED vs old convention)"
    ))
    lines.append("IMU roll: M1-down reads {}  {}".format(
        "NEGATIVE" if m1_imu < 0 else "POSITIVE",
        "(matches old convention)" if imu_ok else "(INVERTED vs old convention)"
    ))
    lines.append("")
    lines.append("Gyro sign: check log.csv rows where ENC_DEG changes rapidly toward")
    lines.append("  M1 restrictor. GYRO_X should be NEGATIVE during that descent.")
    lines.append("  If GYRO_X is POSITIVE during descent -> gyro sign also inverted.")
    lines.append("")

    if enc_ok and imu_ok:
        lines.append("Control chain: no sign changes needed.")
        lines.append("Update AXIS_CENTER to {} in main.py and retune.".format(axis_center))
    elif not imu_ok and enc_ok:
        lines.append("IMU roll inverted. In main.py, negate both imu_roll and gyro_x")
        lines.append("so existing formulas remain unchanged:")
        lines.append("  imu_roll = -degrees(2.0 * atan2(iqi, iqr))")
        lines.append("  gyro_x   = -degrees(gx)")
        lines.append("Update AXIS_CENTER to {}.".format(axis_center))
    elif imu_ok and not enc_ok:
        lines.append("Encoder sign inverted (affects telemetry/plots, not control).")
        lines.append("Negate enc_angle in main.py if you want plots to show correct sign.")
        lines.append("Update AXIS_CENTER to {} in main.py.".format(axis_center))
    else:
        lines.append("Both encoder and IMU signs inverted.")
        lines.append("Control chain signs cancel -- verify gyro carefully before flying.")
        lines.append("Update AXIS_CENTER to {} in main.py.".format(axis_center))

    return lines


def print_summary(axis_center, m1_down, m2_down):
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    for line in _summary_lines(axis_center, m1_down, m2_down):
        print(line)
    print("=" * 50)


def write_summary(run_dir, axis_center, m1_down, m2_down):
    """Write summary.txt alongside log.csv in the run directory."""
    path = run_dir + "/summary.txt"
    f = open(path, "w")
    f.write("Coordinate check summary\n")
    f.write("=" * 50 + "\n")
    for line in _summary_lines(axis_center, m1_down, m2_down):
        f.write(line + "\n")
    f.write("=" * 50 + "\n")
    f.close()


# =====================================================
# Main
# =====================================================

def main():
    i2c = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
    encoder = AS5600(i2c=i2c)

    print("Mounting SD card...")
    f, run_dir = mount_sd_and_open_log(i2c)
    print("Logging to:", run_dir)

    imu = BNO08X_I2C(
        i2c,
        address=0x4A,
        reset_pin=Pin(PIN_IMU_RST, Pin.OUT),
        int_pin=Pin(PIN_IMU_INT, Pin.IN, Pin.PULL_UP),
    )
    imu.game_quaternion.enable(hertz=IMU_REPORT_HZ)
    imu.gyro.enable(hertz=IMU_REPORT_HZ)

    axis_center = 0
    m1_down = None
    m2_down = None

    try:
        # ── Phase A: find AXIS_CENTER ─────────────────────────────────────
        set_led(b=1)
        print("\n=== Phase A: Find AXIS_CENTER ===")
        print("Place lever at PHYSICAL HORIZONTAL (jig or bubble level).")
        print("Press A to capture AXIS_CENTER.")
        print("(ENC_DEG is meaningless until A is pressed.)\n")

        while btn_A.value() == 1:
            t, raw, _, imu_roll, gyro_x = read_sensors(imu, encoder, 0)
            log_row(f, t, "A", raw, 0.0, imu_roll, gyro_x)
            print("  raw={:4d}   imu_roll={:+6.1f}°   gyro={:+6.1f}°/s".format(
                raw, imu_roll, gyro_x), end="\r")
            utime.sleep_ms(_INTERVAL_MS)

        axis_center = encoder.read_raw_angle()
        t, raw, _, imu_roll, gyro_x = read_sensors(imu, encoder, axis_center)
        log_row(f, t, "A", raw, 0.0, imu_roll, gyro_x, "AXIS_CENTER={}".format(axis_center))
        f.flush()
        wait_release(btn_A)
        print("\n  Captured AXIS_CENTER = {}".format(axis_center))

        # ── Phase B: tare IMU ─────────────────────────────────────────────
        print("\n=== Phase B: Tare IMU ===")
        print("Keep lever at horizontal. Press B to tare.\n")

        while btn_B.value() == 1:
            t, raw, enc_deg, imu_roll, gyro_x = read_sensors(imu, encoder, axis_center)
            log_row(f, t, "B", raw, enc_deg, imu_roll, gyro_x)
            print("  enc={:+6.1f}°   imu_roll={:+6.1f}°   gyro={:+6.1f}°/s".format(
                enc_deg, imu_roll, gyro_x), end="\r")
            utime.sleep_ms(_INTERVAL_MS)

        imu.tare(0x07, 1)   # basis=1: Game Rotation Vector — no magnetometer
        t, raw, enc_deg, imu_roll, gyro_x = read_sensors(imu, encoder, axis_center)
        log_row(f, t, "B", raw, enc_deg, imu_roll, gyro_x, "TARE")
        f.flush()
        wait_release(btn_B)
        print("\n  IMU tared.")

        # ── Phase C/D: live readout + mark extremes ───────────────────────
        set_led(g=1)
        print("\n=== Phase C: Mark extremes ===")
        print("Move lever freely and observe signs.")
        print("X = mark M1-end-down  |  Y = mark M2-end-down  |  A+B = exit\n")

        phase = "C"
        while True:
            # Exit: A+B held simultaneously
            if btn_A.value() == 0 and btn_B.value() == 0:
                break

            t, raw, enc_deg, imu_roll, gyro_x = read_sensors(imu, encoder, axis_center)

            if btn_X.value() == 0:
                m1_down = (enc_deg, imu_roll, gyro_x)
                log_row(f, t, "D", raw, enc_deg, imu_roll, gyro_x, "M1_DOWN")
                f.flush()
                wait_release(btn_X)
                phase = "D"
                print("\n  M1-down marked  enc={:+.1f}°  imu_roll={:+.1f}°".format(enc_deg, imu_roll))
                continue

            if btn_Y.value() == 0:
                m2_down = (enc_deg, imu_roll, gyro_x)
                log_row(f, t, "D", raw, enc_deg, imu_roll, gyro_x, "M2_DOWN")
                f.flush()
                wait_release(btn_Y)
                phase = "D"
                print("\n  M2-down marked  enc={:+.1f}°  imu_roll={:+.1f}°".format(enc_deg, imu_roll))
                continue

            log_row(f, t, phase, raw, enc_deg, imu_roll, gyro_x)
            print("  enc={:+6.2f}°   imu_roll={:+6.2f}°   gyro={:+6.2f}°/s   [X=M1↓  Y=M2↓  A+B=exit]".format(
                enc_deg, imu_roll, gyro_x), end="\r")
            utime.sleep_ms(_INTERVAL_MS)

        f.flush()
        write_summary(run_dir, axis_center, m1_down, m2_down)

    finally:
        f.close()
        os.umount(_SD_MOUNT)
        set_led(b=1)
        print("\nSD card unmounted.")

    print_summary(axis_center, m1_down, m2_down)


main()