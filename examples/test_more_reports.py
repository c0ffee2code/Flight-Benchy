# test_more_reports.py
#
# BNO08x Micropython I2C example program
# Steps counted,  Stability classifier, Activity classifier

from time import sleep

from adafruit.imu.bradcar.bno08x import *
from adafruit.imu.bradcar.i2c import BNO08X_I2C
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff

int_pin = Pin(3, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
reset_pin = Pin(2, Pin.OUT)  # BNO sensor (RST)

address = 0x4a
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
print(f"I2C {hex(address)} found" if address in i2c0.scan() else f"ERROR: I2C not configured")

bno = BNO08X_I2C(i2c0, address=address, reset_pin=reset_pin, int_pin=int_pin)
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

print("Start")
print("====================================\n")

bno.steps.enable()
bno.stability_classifier.enable()
bno.activity_classifier.enable()

bno.print_report_period()

last_print = ticks_ms()
print("\nStart loop:")
while True:    
    # Required each loop to refresh sensor data
    bno.update_sensors()

    # print out results every 0.5 sec (500 ms)
    now = ticks_ms()
    if ticks_diff(now, last_print) >= 500:
        last_print = now

        print(f"\nTotal Steps detected: {bno.steps}")
        print(f"Stability classifier: {bno.stability_classifier}")

        activity, confidence = bno.activity_classifier
        print(f"Activity classifier: {activity}, confidence: {confidence}%")
