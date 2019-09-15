# This is the main file for ESP32 MicroPython board
from cpython_lora_range_test.machine import I2C, SPI, Pin
from esp32_lora_range_test.spicontrol import SpiControl
from drivers.sx1278 import SX1278
from drivers.oled_display import OledDisp


def main_func():
    # OLED I2C module interface
    i2c = I2C(scl=Pin(15), sda=Pin(4))
    oled_rst_n = Pin(16, Pin.OUT)
    oled_rst_n.on()  # Release reset on OLED module
    oled_display = OledDisp(i2c)

    # LoRa SPI interface
    spi = SPI(baudrate=10000000, polarity=0, phase=0, sck=Pin(5), mosi=Pin(27), miso=Pin(19))
    spi_ctrl = SpiControl(spi, Pin(18, Pin.OUT))
    lora_rst_n = Pin(14, Pin.OUT)
    lora_rst_n.on()  # Release reset on LoRa module

    # Blue LED interface
    blue_led = Pin(2, Pin.OUT)

    # Button interface
    button = Pin(0, Pin.IN)

    oled_display.text_to_row("Hello world!", 0)
    oled_display.update()

    exit()

    # SX1278 LoRa module
    lora = SX1278(spi_ctrl, parameters={'frequency': 433.92e6,
                                        'pa_select': 0,
                                        'spreading_factor': 12,
                                        'bandwidth': '7.8 kHz'})

    lora.print_chip_rev()

    lora_esp32(lora, oled_display, button, blue_led)


if __name__ == '__main__':
    main_func()
