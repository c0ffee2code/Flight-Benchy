from picographics import PicoGraphics, DISPLAY_PICO_DISPLAY
from micropython import const

# https://shop.pimoroni.com/products/pico-display-pack
# https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/pico_display
# https://github.com/UnfinishedStuff/Pimoroni_Pico_Display_Pack_documentation

# Init display with rotation (landscape mode)
display = PicoGraphics(display=DISPLAY_PICO_DISPLAY, rotate=180)
display.set_backlight(1)

# Define pens
black = display.create_pen(0, 0, 0)
white = display.create_pen(255, 255, 255)

# Disclose screen dimensions
# Given that I only have PICO Display pack, it is 240 x 135
WIDTH, HEIGHT = display.get_bounds()
print(f"{WIDTH} x {HEIGHT}")

# Fonts
# bitmap8 font has height of 8 pixels
# bitmap fonts are aligned from their top left corner
display.set_font("bitmap8")
# with scale set to 3 every symbol has 24 pixels height
SCALE = const(3)

#
X_COL_1 = const(0)
X_COL_2 = const(105)

# Screen has height of 135 pixels
# with scale of 3 every character has 24 pixels height
# I plan to display 3 rows of information, 135/3 = 45 pixels per row
# first row starts at 0
Y_ROW_1 = const(0)
# center row 45 + ((45 − 24) / 2) ~= 56
Y_ROW_2 = const(56)
# last row is 135 - 24 = 111
Y_ROW_3 = const(111)

def draw_screen(encoder_angle, imu_angle, angle_diff, lag_ms):
    display.set_pen(black)
    display.clear()
    display.set_pen(white)

    # 1st row
    display.text("REF:", X_COL_1, Y_ROW_1, scale=SCALE, fixed_width=True)
    display.text("{:.2f}°".format(encoder_angle), X_COL_2, Y_ROW_1, scale=SCALE, fixed_width=True)

    # 2nd row
    display.text("IMU:", X_COL_1, Y_ROW_2, scale=SCALE, fixed_width=True)
    display.text("{:.2f}°".format(imu_angle), X_COL_2, Y_ROW_2, scale=SCALE, fixed_width=True)

    # 3rd row
    display.text("{:.2f}".format(angle_diff), X_COL_1, Y_ROW_3, scale=SCALE, fixed_width=True)
    display.text("{:.2f}".format(lag_ms), X_COL_2, Y_ROW_3, scale=SCALE, fixed_width=True)

    display.update()
