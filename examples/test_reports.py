# test_reports.py
#
# BNO08x MicroPython I2C Test
#
# I2C interface: Test common sensor reports:
# acceleration, magnetic, gryoscope, quaternion, quaternion.euler
#
# Enabling reports at 4 Hz (~0.25 sec)
# sensor provides frequencies close to what was requested

from adafruit.imu.bradcar.bno08x import *
from adafruit.imu.bradcar.i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms

int_pin = Pin(3, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(2, Pin.OUT)  # BNO sensor (RST)

address = 0x4a
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

print("Start")
print("====================================\n")

bno.acceleration.enable(4)
bno.magnetic.enable(4)
bno.gyro.enable(4)
bno.quaternion.enable(4)

bno.print_report_period()

print("\nStart loop:")
while True:
    # Update required each loop to check if any sensor updated, print sensor data (some or all may be old data)
    # see test_reports_full.py, for example of only printing a sensor when it is updated
    bno.update_sensors()

    print(f"\nsystem {ticks_ms()=}")

    accel_x, accel_y, accel_z = bno.acceleration
    print(f"\nAcceleration X: {accel_x:+.3f}  Y: {accel_y:+.3f}  Z: {accel_z:+.3f}  m/s²")

    mag_x, mag_y, mag_z = bno.magnetic
    print(f"Magnetometer X: {mag_x:+.3f}  Y: {mag_y:+.3f}  Z: {mag_z:+.3f}  uT")

    gyro_x, gyro_y, gyro_z = bno.gyro
    print(f"Gyroscope    X: {gyro_x:+.3f}  Y: {gyro_y:+.3f}  Z: {gyro_z:+.3f}  rads/s")

    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"Quaternion   I: {quat_i:+.3f}  J: {quat_j:+.3f}  K: {quat_k:+.3f}  Real: {quat_real:+.3f}")

    roll, pitch, yaw = bno.quaternion.euler
    print(f"Euler Angle: Roll {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
