from picographics import PicoGraphics, DISPLAY_PICO_DISPLAY
from micropython import const

# https://shop.pimoroni.com/products/pico-display-pack
# https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/pico_display

# Init display with rotation (landscape mode)
display = PicoGraphics(display=DISPLAY_PICO_DISPLAY, rotate=180)
display.set_backlight(1)

# Define pens
black = display.create_pen(0, 0, 0)
white = display.create_pen(255, 255, 255)
green = display.create_pen(0, 255, 0)
red = display.create_pen(255, 0, 0)

# Screen dimensions: 240 x 135
WIDTH, HEIGHT = display.get_bounds()

# Fonts
# bitmap8 font has height of 8 pixels
# bitmap fonts are aligned from their top left corner
display.set_font("bitmap8")
# with scale set to 3 every symbol has 24 pixels height
SCALE = const(3)

X_COL_1 = const(0)
X_COL_2 = const(105)

# Screen has height of 135 pixels
# with scale of 3 every character has 24 pixels height
# 3 rows of information, 135/3 = 45 pixels per row
Y_ROW_1 = const(0)
Y_ROW_2 = const(56)
Y_ROW_3 = const(111)


def _clear():
    display.set_pen(black)
    display.clear()


def draw_disarmed():
    _clear()
    display.set_pen(white)
    display.text("DISARMED", X_COL_1, Y_ROW_1, scale=SCALE)
    display.text("Hold B+Y", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("to ARM", X_COL_1, Y_ROW_3, scale=SCALE)
    display.update()


def draw_arming():
    _clear()
    display.set_pen(green)
    display.text("ARMING", X_COL_1, Y_ROW_1, scale=SCALE)
    display.set_pen(white)
    display.text("Please", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("wait...", X_COL_1, Y_ROW_3, scale=SCALE)
    display.update()


def draw_ready(angle):
    _clear()
    display.set_pen(white)
    display.text("{:+.1f}".format(angle), X_COL_1, Y_ROW_1, scale=SCALE)
    display.set_pen(green)
    display.text("Press A", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("to START", X_COL_1, Y_ROW_3, scale=SCALE)
    display.update()


def draw_stabilizing(angle, m1, m2):
    _clear()
    display.set_pen(white)
    display.text("{:+.1f}".format(angle), X_COL_1, Y_ROW_1, scale=SCALE)

    display.text("M1:", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text(str(m1), X_COL_2, Y_ROW_2, scale=SCALE)

    display.text("M2:", X_COL_1, Y_ROW_3, scale=SCALE)
    display.text(str(m2), X_COL_2, Y_ROW_3, scale=SCALE)
    display.update()


def draw_error(msg):
    _clear()
    display.set_pen(red)
    display.text("ERROR", X_COL_1, Y_ROW_1, scale=SCALE)
    display.set_pen(white)
    display.text(msg[:12], X_COL_1, Y_ROW_2, scale=SCALE)
    display.update()
