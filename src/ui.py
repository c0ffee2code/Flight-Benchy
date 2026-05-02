# Operator interface — hardware: Pimoroni Pico Display Pack
# https://shop.pimoroni.com/products/pico-display-pack
# Buttons (GPIO 12–15) and RGB LED (GPIO 6/7/8) from the Display Pack are used.
# LCD is physically disconnected (SPI0 conflict with Adalogger SD card, see DR-004).

from machine import Pin
from micropython import const
import utime

# RGB LED (active HIGH, common cathode — value 1 = LED on)
PIN_LED_R = const(6)    # Red — error
PIN_LED_G = const(7)    # Green — armed / stabilizing
PIN_LED_B = const(8)    # Blue — idle / ready to arm

# Buttons (active LOW — PULL_UP, pressed = 0)
PIN_BTN_A = const(12)
PIN_BTN_B = const(13)
PIN_BTN_X = const(14)
PIN_BTN_Y = const(15)

led_r = Pin(PIN_LED_R, Pin.OUT)
led_g = Pin(PIN_LED_G, Pin.OUT)
led_b = Pin(PIN_LED_B, Pin.OUT)

btn_A = Pin(PIN_BTN_A, Pin.IN, Pin.PULL_UP)
btn_B = Pin(PIN_BTN_B, Pin.IN, Pin.PULL_UP)
btn_X = Pin(PIN_BTN_X, Pin.IN, Pin.PULL_UP)
btn_Y = Pin(PIN_BTN_Y, Pin.IN, Pin.PULL_UP)


def set_led(r=0, g=0, b=0):
    """Set RGB LED state. Active HIGH — pass 1 to turn a channel on."""
    led_r.value(not r)
    led_g.value(not g)
    led_b.value(not b)


def buttons_by_held():
    """Return True if B+Y are both pressed (active low)."""
    return not btn_B.value() and not btn_Y.value()


def wait_for_go():
    """Blue LED, block until B+Y held."""
    set_led(b=1)
    while not buttons_by_held():
        utime.sleep_ms(10)