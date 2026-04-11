"""
Open-loop single-motor encoder sweep for mechanical characterisation.

Disconnect the inactive motor from its ESC. Run this script via REPL,
follow the prompts, and the active motor sweeps through a throttle range
while the encoder is sampled at 200 Hz. Results are written to SD card.

Interaction model: REPL / serial terminal (no buttons required).
  1. Disconnect the inactive motor from its ESC.
  2. Connect via USB serial and run the script.
  3. Select the active motor at the prompt.
  4. For each throttle step: position lever at the start restrictor, press ENTER.
     The motor runs for RUN_SECS while the encoder is sampled; data is
     written to SD card and a one-line summary is printed.

Output: /sd/bench_sweep/YYYY-MM-DD_hh-mm-ss/
  config.yaml         — session parameters
  throttle_NNN.csv    — one file per throttle step (T_MS, THROTTLE, ENC_DEG, ENC_RAW)
"""

from micropython import const
from machine import I2C, Pin, SPI
import os
import time
import utime

from as5600 import AS5600, to_degrees
from motor_throttle_group import MotorThrottleGroup
from dshot_pio import DSHOT_SPEEDS
import sdcard

# ── Hardware ──────────────────────────────────────────────────────────
PIN_I2C_SDA  = const(0)
PIN_I2C_SCL  = const(1)
PIN_MOTOR1   = const(10)
PIN_MOTOR2   = const(11)
PIN_SD_MISO  = const(16)
PIN_SD_CS    = const(17)
PIN_SD_SCK   = const(18)
PIN_SD_MOSI  = const(19)

# ── Calibration ───────────────────────────────────────────────────────
AXIS_CENTER  = const(275)

# ── Motor limits ──────────────────────────────────────────────────────
THROTTLE_MIN = const(100)

# ── Sweep parameters ──────────────────────────────────────────────────
THROTTLE_START = const(200)
THROTTLE_END   = const(300)
THROTTLE_STEP  = const(10)
RUN_SECS       = const(10)
SAMPLE_MS      = const(5)    # 200 Hz

# ── SD card paths ─────────────────────────────────────────────────────
SD_MOUNT  = "/sd"
SWEEP_DIR = SD_MOUNT + "/bench_sweep"


def bcd(b):
    """Decode a packed BCD byte to an integer (e.g. 0x23 → 23)."""
    return (b >> 4) * 10 + (b & 0x0F)


def read_rtc(i2c, addr=0x68):
    """Read current time from PCF8523 RTC.

    Returns (year, month, day, hour, minute, second).
    Raises OSError if the oscillator-stop flag is set (clock not reliable).
    """
    buf = i2c.readfrom_mem(addr, 0x03, 7)
    if buf[0] & 0x80:
        raise OSError("PCF8523: OS flag set — re-run set_rtc.py")
    return (
        bcd(buf[6]) + 2000,
        bcd(buf[5] & 0x1F),
        bcd(buf[3] & 0x3F),
        bcd(buf[2] & 0x3F),
        bcd(buf[1] & 0x7F),
        bcd(buf[0] & 0x7F),
    )


def mount_sd():
    """Mount SD card. Returns (sd, vfs) — caller must hold references to prevent GC.

    Unmounts first if already mounted (e.g. after a crash or prior run without reboot).
    """
    try:
        os.umount(SD_MOUNT)
    except OSError:
        pass  # not mounted — expected on a clean boot
    cs_pin = Pin(PIN_SD_CS, Pin.OUT, value=1)
    spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
              sck=Pin(PIN_SD_SCK), mosi=Pin(PIN_SD_MOSI), miso=Pin(PIN_SD_MISO))
    time.sleep_ms(250)
    sd = sdcard.SDCard(spi, cs_pin)
    vfs = os.VfsFat(sd)
    os.mount(vfs, SD_MOUNT)
    return sd, vfs


def setup_session(dt, motor_label):
    """Create the timestamped session directory and write config.yaml.

    Returns the session directory path.
    """
    try:
        os.mkdir(SWEEP_DIR)
    except OSError:
        pass  # already exists
    run_dir = "{}/{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
        SWEEP_DIR, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]
    )
    os.mkdir(run_dir)

    config = (
        "active_motor: {motor}\n"
        "throttle_start: {t_start}\n"
        "throttle_end: {t_end}\n"
        "throttle_step: {t_step}\n"
        "run_secs: {secs}\n"
        "sample_ms: {samp}\n"
        "throttle_min: {t_min}\n"
        "axis_center: {center}\n"
    ).format(
        motor=motor_label,
        t_start=THROTTLE_START, t_end=THROTTLE_END, t_step=THROTTLE_STEP,
        secs=RUN_SECS, samp=SAMPLE_MS, t_min=THROTTLE_MIN, center=AXIS_CENTER,
    )
    with open(run_dir + "/config.yaml", "w") as f:
        f.write(config)

    return run_dir


def arm_motors(motors):
    """Start DShot, arm ESCs, and hold at minimum throttle until arming beeps settle."""
    motors.start()
    motors.arm()
    motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])
    utime.sleep_ms(1500)


def sample_encoder(motors, encoder, active_motor, throttle):
    """Apply throttle and sample the encoder at SAMPLE_MS intervals for RUN_SECS.

    Data is collected entirely in memory — SD writes happen after the timed window
    to avoid I/O latency perturbing the sample rate.

    Returns (buf_t, buf_deg, buf_raw): parallel lists of timestamps (ms),
    angles (degrees), and raw encoder counts.
    """
    motors.setThrottle(active_motor, throttle)

    buf_t   = []
    buf_deg = []
    buf_raw = []

    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < RUN_SECS * 1000:
        tick = utime.ticks_ms()
        raw  = encoder.read_raw_angle()
        deg  = to_degrees(raw, AXIS_CENTER)
        buf_t.append(utime.ticks_diff(tick, start))
        buf_deg.append(deg)
        buf_raw.append(raw)
        elapsed = utime.ticks_diff(utime.ticks_ms(), tick)
        if elapsed < SAMPLE_MS:
            utime.sleep_ms(SAMPLE_MS - elapsed)

    motors.setThrottle(active_motor, THROTTLE_MIN)
    return buf_t, buf_deg, buf_raw


def write_step_csv(run_dir, throttle, buf_t, buf_deg, buf_raw):
    """Write one throttle step's encoder data to throttle_NNN.csv."""
    fname = run_dir + "/throttle_{:03d}.csv".format(throttle)
    with open(fname, "w") as f:
        f.write("T_MS,THROTTLE,ENC_DEG,ENC_RAW\n")
        for i in range(len(buf_t)):
            f.write("{},{},{:.2f},{}\n".format(buf_t[i], throttle, buf_deg[i], buf_raw[i]))


def run_step(motors, encoder, active_motor, throttle, run_dir):
    """Run one throttle step: prompt for lever position, sample, write, print summary."""
    input("\nThrottle {}: hold lever at start position, then press ENTER.".format(throttle))

    buf_t, buf_deg, buf_raw = sample_encoder(motors, encoder, active_motor, throttle)
    write_step_csv(run_dir, throttle, buf_t, buf_deg, buf_raw)

    start_deg = buf_deg[0]  if buf_deg else 0.0
    end_deg   = buf_deg[-1] if buf_deg else 0.0
    print("  Start: {:.1f}°  End: {:.1f}°  Travel: {:+.1f}°  ({} samples)".format(
        start_deg, end_deg, end_deg - start_deg, len(buf_t)
    ))


def main():
    i2c     = I2C(0, scl=Pin(PIN_I2C_SCL), sda=Pin(PIN_I2C_SDA), freq=400_000)
    encoder = AS5600(i2c=i2c)
    motors  = MotorThrottleGroup([Pin(PIN_MOTOR1), Pin(PIN_MOTOR2)], DSHOT_SPEEDS.DSHOT600)

    # Mount SD before arming — fail fast if card is absent
    sd, vfs = mount_sd()
    dt = read_rtc(i2c)

    print("\nBench sweep — open-loop encoder sweep for mechanical characterisation.")
    print("Disconnect the INACTIVE motor from its ESC before proceeding.\n")
    sel = input("Active motor (1 = M1 / GPIO10, 2 = M2 / GPIO11): ").strip()
    if sel not in ("1", "2"):
        print("Invalid selection. Exiting.")
        os.umount(SD_MOUNT)
        return
    active_motor = int(sel) - 1   # 0-indexed for MotorThrottleGroup
    motor_label  = "M{}".format(sel)

    run_dir = setup_session(dt, motor_label)
    print("\nSession: {}".format(run_dir))
    print("Motor: {}  |  Range: {}–{}  step {}  |  {}s / step at {}Hz".format(
        motor_label,
        THROTTLE_START, THROTTLE_END, THROTTLE_STEP,
        RUN_SECS, 1000 // SAMPLE_MS,
    ))

    arm_motors(motors)
    print("Motors armed.")

    for throttle in range(THROTTLE_START, THROTTLE_END + 1, THROTTLE_STEP):
        run_step(motors, encoder, active_motor, throttle, run_dir)

    motors.setAllThrottles([THROTTLE_MIN, THROTTLE_MIN])
    utime.sleep_ms(500)
    motors.disarm()
    motors.stop()

    os.umount(SD_MOUNT)
    print("\nSweep complete. Data saved to: {}".format(run_dir))


main()