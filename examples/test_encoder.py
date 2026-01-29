from micropython import const
from machine import I2C, Pin
import time
from adafruit.encoder.as5600 import AS5600, to_degrees

i2c = I2C(
    0,
    scl=Pin(1),
    sda=Pin(0),
    freq=400_000
)

encoder = AS5600(i2c=i2c, debug=False)

# recorded while i was holding lever at horizontal position
# has to be recaptured each time mechanical part is modified !
AXIS_CENTER = const(422)

while True:
    raw_angle = encoder.read_raw_angle()

    angle =  to_degrees(raw_angle, AXIS_CENTER)

    print("RAW: ", raw_angle, " RELATIVE: ", angle)
    encoder.status()
    time.sleep(0.05)
