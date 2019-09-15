# SPI Commands
# To be used with SPIDriver controller


class SpiControl:
    def __init__(self, spi, _ss_pin=None):
        self._spi = spi
        self._spi.unsel()

    # Transfer data to and from device
    def transfer(self, reg, value=0x00):
        self._spi.sel()
        self._spi.write([reg])
        response = self._spi.writeread(bytearray([value]))
        self._spi.unsel()
        return response

    def read_bytes(self, reg, length):
        self._spi.sel()
        self._spi.write([reg])
        response = self._spi.read(length)
        self._spi.unsel()
        return response

    def write_bytes(self, reg, value):
        self._spi.sel()
        self._spi.write([reg | 0x80])
        self._spi.writeread(value)
        self._spi.unsel()
