# 0.96" OLED display 'driver'
from drivers import ssd1306


class OLEDDisplay:
    def __init__(self, i2c):
        self.display = ssd1306.SSD1306_I2C(128, 64, i2c)

    ''' Write text on row 0-7 '''
    def text_to_row(self, s, row):
        self.clear_row(row)
        self.display.text(s, 0, row * 8, 1)

    ''' Clear row '''
    def clear_row(self, row):
        self.display.fill_rect(0, row * 8, 128, row * 8 + 8, 0)

    ''' Update display '''
    def update(self):
        self.display.show()
