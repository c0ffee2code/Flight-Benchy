"""
Tare calibration — before/after bias comparison

Two-phase procedure:

  PHASE 1 — Attach to bench + settle
    Attach sensor at zero position.
    Wait SETTLE_SECS for Game Rotation Vector to stabilise from accelerometer,
    then collect pre-tare readings.

  PHASE 2 — Tare + verify
    Tare is applied (Game Rotation Vector basis — no magnetometer required).
    Post-tare readings are collected and compared.
    Optionally persist the tare offset to BNO085 flash.

Why Game Rotation Vector basis for tare:
  imu.tare(axes, basis=1) targets the Game Rotation Vector (GRV), which is
  driven by accelerometer + gyroscope only.  No magnetometer is involved.
  This avoids reference-frame corruption in environments where the local magnetic
  field differs from the calibration environment (near motors, ESCs, metal
  structures, or after moving the device to a new room).

  With basis=0 (Rotation Vector) the tare bakes in the current magnetometer
  heading.  If mag accuracy is 0 on the bench, that heading is arbitrary and
  the tare produces a ~45 deg tilted reference frame — observed in testing.

  For yaw accuracy (heading), recalibrate the magnetometer in the operating
  environment with tests/calibration/test_calibration_mag.py and switch
  basis back to 0.

  Spec reference: Tare Usage Guide p2–3, SH-2 §6.4.4.1.

Output CSVs (on Pico flash):
  /data/tare_before.csv  — pre-tare IMU vs encoder
  /data/tare_after.csv   — post-tare IMU vs encoder

References:
  - specification/BNO080-BNO085-Tare-Function-Usage-Guide.pdf
  - specification/IMU BNO08x v1.17.pdf, Section 4.1.1
"""

from math import atan2
from micropython import const
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C

# === Configuration ===
RATE_HZ = const(344)
SAMPLES_PER_PHASE = const(2000)
AXIS_CENTER = const(2378)  # raw reading when lever is at physical zero (was 422, corrected from tare run)
SETTLE_SECS = const(30)

# === Ensure output directory exists ===
try:
    import os
    os.mkdir("/data")
except OSError:
    pass

RAD2DEG = 57.2957795

def quat_to_roll(qr, qi):
    """Roll in degrees from quaternion, matching the formula used by the control loop."""
    return 2.0 * atan2(qi, qr) * RAD2DEG


# === Hardware Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
encoder = AS5600(i2c=i2c)
reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=True)


def wait_for_enter(message):
    """Print instructions and drain sensor packets until user presses ENTER."""
    import select, sys
    print(message)
    while True:
        imu.update_sensors()
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return


# =========================================================================
# PHASE 1: Sensor on bench — enable, settle + collect pre-tare
# =========================================================================
wait_for_enter(
    "\n=== Phase 1: Attach to bench ===\n"
    "  Attach the sensor at the ZERO position.\n"
    "  Press ENTER when attached and stable.")

imu.game_quaternion.enable(RATE_HZ)
imu.begin_calibration()

print(f"\nWaiting {SETTLE_SECS}s for Game Rotation Vector to stabilise.")
print("Keep the lever fixed at zero.\n")

settle_start = ticks_ms()
last_print = ticks_ms()
last_grv_ts = 0.0
while ticks_diff(ticks_ms(), settle_start) < SETTLE_SECS * 1000:
    imu.update_sensors()
    r = imu.game_quaternion.get()
    if r.sensor_ts_ms != last_grv_ts and ticks_diff(ticks_ms(), last_print) >= 500:
        last_grv_ts = r.sensor_ts_ms
        last_print = ticks_ms()
        roll = quat_to_roll(r.data[0], r.data[1])
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        remaining = (SETTLE_SECS * 1000 - ticks_diff(ticks_ms(), settle_start)) / 1000
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  acc: {r.accuracy}  ({remaining:.0f}s left)")


def collect_samples(output_file):
    """Collect SAMPLES_PER_PHASE readings into a CSV file."""
    start_ms = ticks_ms()
    count = 0
    i2c_errors = 0
    last_sample_ts = 0.0
    f = None

    try:
        f = open(output_file, "w")
        f.write("T,ENC,IMU_ROLL,IMU_ACC,Lag,ENC_RAW\n")
        while count < SAMPLES_PER_PHASE:
            now_ms = ticks_ms()
            raw_angle = encoder.read_raw_angle()
            encoder_angle = to_degrees(raw_angle, AXIS_CENTER)

            try:
                imu.update_sensors()
            except OSError:
                i2c_errors += 1
                if i2c_errors > 10:
                    print(f"\n  Too many I2C errors ({i2c_errors}), aborting collection.")
                    break
                sleep_ms(10)
                continue

            r = imu.game_quaternion.get()
            if r.sensor_ts_ms != last_sample_ts:
                last_sample_ts = r.sensor_ts_ms
                roll = quat_to_roll(r.data[0], r.data[1])
                lag = imu.bno_start_diff(r.host_ts_ms) - r.sensor_ts_ms
                elapsed = ticks_diff(now_ms, start_ms)
                f.write(f"{elapsed},{encoder_angle:.2f},{roll:.2f},{r.accuracy},{lag:.1f},{raw_angle}\n")
                count += 1
    except KeyboardInterrupt:
        pass
    finally:
        if f is not None:
            f.close()

    elapsed_s = ticks_diff(ticks_ms(), start_ms) / 1000.0
    hz = count / elapsed_s if elapsed_s > 0 else 0
    if i2c_errors:
        print(f"  WARNING: {i2c_errors} I2C error(s) during collection")
    print(f"  {count} samples in {elapsed_s:.1f}s ({hz:.1f} Hz) -> {output_file}")
    return count


print("\n--- Pre-tare readings ---")
print("Collecting readings with current (untared) orientation...\n")
collect_samples("/data/tare_before.csv")


# =========================================================================
# PHASE 2: Apply tare + collect post-tare
# =========================================================================
imu.update_sensors()
enc_at_tare = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
r = imu.game_quaternion.get()
if r.sensor_ts_ms != last_grv_ts:
    roll_at_tare = quat_to_roll(r.data[0], r.data[1])
    print(f"\nAt tare moment --- ENC: {enc_at_tare:+.2f}  IMU roll: {roll_at_tare:+.2f}  bias: {roll_at_tare - enc_at_tare:+.2f}")
else:
    print(f"\nAt tare moment --- ENC: {enc_at_tare:+.2f}  (IMU not updated)")

print("Applying tare (all axes, Game Rotation Vector basis — no mag required)...")
imu.tare(0x07, 1)

# Drain any packets the tare command may have triggered before collecting data.
# Insufficient settling here can leave the I2C bus in a bad state.
for _ in range(50):
    imu.update_sensors()
    sleep_ms(10)

print("Tare applied. Post-tare check:")
last_grv_ts_post = 0.0
for _ in range(5):
    imu.update_sensors()
    r = imu.game_quaternion.get()
    if r.sensor_ts_ms != last_grv_ts_post:
        last_grv_ts_post = r.sensor_ts_ms
        roll = quat_to_roll(r.data[0], r.data[1])
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}")
    sleep_ms(200)

print("\n--- Post-tare readings ---")
print("Collecting readings with tared orientation...\n")
collect_samples("/data/tare_after.csv")

# === Persist tare ===
save = input("\nSave tare to BNO085 flash? [y/N]: ").strip().lower()
if save == "y":
    imu.save_tare_data()
    print("Tare saved (survives power cycle).")
else:
    print("Tare NOT saved (lost on power cycle).")

print("\nDone. Retrieve files with:")
print("  mpremote cp :/data/tare_before.csv .")
print("  mpremote cp :/data/tare_after.csv .")
print("\nCompare with:")
print("  python analyse_report_rate.py tare_before.csv tare_after.csv")