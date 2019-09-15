# SPI Commands
# To be used with MicroPython SPI


class SpiControl:
    def __init__(self, spi, ss_pin):
        self._spi = spi
        self._ss_pin = ss_pin
        self._ss_pin.value(1)

    # Transfer data to and from device
    def transfer(self, reg, value=0x00):
        response = bytearray(1)
        self._ss_pin.value(0)
        self._spi.write(bytearray([reg]))
        self._spi.write_readinto(bytearray([value]), response)
        self._ss_pin.value(1)
        return response

    def read_bytes(self, reg, length):
        response = bytearray(length)
        self._ss_pin.value(0)
        self._spi.write(bytearray([reg]))
        self._spi.readinto(response)
        self._ss_pin.value(1)
        return response

    def write_bytes(self, reg, value):
        self._ss_pin.value(0)
        self._spi.write(bytearray([reg | 0x80]))
        self._spi.write(value)
        self._ss_pin.value(1)
