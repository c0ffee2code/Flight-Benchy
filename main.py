from micropython import const
from machine import I2C, Pin, Timer
import time
from utime import ticks_ms
from pimoroni.display.display_pack import draw_screen
from adafruit.encoder.as5600 import AS5600, to_degrees
from adafruit.imu.bradcar.i2c import BNO08X_I2C

i2c = I2C(
    0,
    scl=Pin(1),
    sda=Pin(0),
    freq=400_000
)

# Magnetic encoder AS5600
encoder = AS5600(i2c=i2c, debug=False)
# recorded while i was holding lever at horizontal position
# has to be recaptured each time mechanical part is modified !
AXIS_CENTER = const(422)

# Inertial Measurement Unit BNO085
reset_pin = Pin(2, Pin.OUT)  # BNO sensor (RST)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)  # BNO sensor (INT)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)
imu.quaternion.enable(20)

# Pimoroni Pico Display Pack
DISPLAY_LOOP_MS = const(100)
last_control_update = time.ticks_ms()
last_print_time = time.ticks_ms()

# Values to display
output_available = False
encoder_angle = None
imu_angle = None
angle_diff = None
lag_ms = None

def update_display(timer):
    """Timer callback to update display"""
    if output_available:
        draw_screen(encoder_angle, imu_angle, angle_diff, lag_ms)

display_timer = Timer()
display_timer.init(period=DISPLAY_LOOP_MS, mode=Timer.PERIODIC, callback=update_display)

try:
    while True:
        ms_current = ticks_ms()
        # Record current position via Encoder
        encoder_angle =  to_degrees(encoder.read_raw_angle(), AXIS_CENTER)

        # Update required each loop to check if any sensor updated, print timestamp if any sensor was updated
        if imu.update_sensors() > 0:
            ms_since_sensor_start = imu.bno_start_diff(ms_current)
            # print(f"\nsystem {ms_current=},",
            #       f"time from BNO start: {ms_since_sensor_start / 1000.0:.3f} s",
            #       f"({ms_since_sensor_start:.0f} ms)")

        if imu.quaternion.updated:
            roll, pitch, yaw, acc, ts_ms = imu.quaternion.euler_full
            imu_angle = pitch
            # print(f"Euler Angle: Roll {roll:+.3f}°  Pitch: {pitch:+.3f}°  Yaw: {yaw:+.3f}°  degrees")
            # print(f"Euler Angle: accuracy={acc}, {ts_ms=:.1f}")

            angle_diff = encoder_angle - imu_angle
            print(f"Angle diff: {angle_diff:+.3f}°")

            imu_now_ms = imu.bno_start_diff(ms_current)
            lag_ms = imu_now_ms - ts_ms
            print(f"Lag: {lag_ms:+.3f} ms")

            output_available = True

finally:
    display_timer.deinit()

