# test_freq.py
#
# BNO08x MicroPython I2C Test
#
# Test update report frequency of quaternion.full
# We only print average of report period every 500 iterations
#
# Enabling reports at 400Hz (2.5ms/report), I2C can loop at up to 4.8ms.
# At higher frequencies, since controller falls behind the sensor packages multple reports together
# but only returns latest so apparent period appears longer.

from time import sleep

from adafruit.imu.bradcar.i2c import BNO08X_I2C
from adafruit.imu.bradcar.bno08x import *

from machine import I2C, Pin
from utime import ticks_ms, sleep_us

int_pin = Pin(3, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(2, Pin.OUT)  # BNO sensor (RST)

address = 0x4a
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])
print("Start")

print("====================================\n")

bno.quaternion.enable(400)
#bno.acceleration.enable(250)
bno.print_report_period()

# Every sum_count print average duration
sum_count=500
count = 0
running_sum = 0.0

print("\nStart loop:")

# time of 1st quaternion
bno.update_sensors()
_, _, _, _, _, last_ts_ms = bno.quaternion.full
#x, y, z, _, last_ts_ms = bno.acceleration.full


while True:
    # required to get data from enabled sensors
    while bno.update_sensors() == 0:
        pass
    
    # Get Quaternion report, in this test, we are not using report values, but could log them to array
    quat_i, quat_j, quat_k, quat_real, _, ts_ms = bno.quaternion.full
    #x, y, z, _, ts_ms = bno.acceleration.full

    running_sum += ts_ms - last_ts_ms
    count +=1
    last_ts_ms = ts_ms
        
    if count % sum_count == 0:
        print(f"ave={running_sum/sum_count:.1f}")
        running_sum=0.0
