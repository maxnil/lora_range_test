#!/usr/bin/env python3

# This is the main file for the Laptop version of the script
# It uses the USB SPIDriver controller
from cpython_lora_range_test.machine import SPI
from cpython_lora_range_test.spicontrol import SpiControl
from drivers.sx1278 import SX1278


def main_func():
    # LoRa SPI interface and SPI Control
    spi = SPI()
    spi_ctrl = SpiControl(spi)

    # SX1278 LoRa module
    lora = SX1278(spi_ctrl, parameters={'frequency': 433.92e6,
                                        'pa_select': 0,
                                        'spreading_factor': 12,
                                        'bandwidth': '7.8 kHz'})

    lora.print_chip_rev()

    # lora_laptop(lora)


if __name__ == '__main__':
    main_func()
