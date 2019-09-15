# Wrapper for I2CDriver, SPIDriver and Pin to work as MicroPython's I2C, SPI and Pin drivers
from i2cdriver import I2CDriver
from spidriver import SPIDriver
import serial


class I2C:
    I2CDRIVER_COM_PORT = "/dev/cu.usbserial-DO01JHNA"

    def __init__(self, _scl, _sda):
        try:
            self.i2c = I2CDriver(self.I2CDRIVER_COM_PORT)
            self.i2c.setspeed(400)
        except serial.serialutil.SerialException as exc:
            print("###ERR: Failed to connect to I2CDriver controller")
            print(f"###ERR:   {exc}")
            exit()

    def writeto(self, addr, buf, stop=True):
        self.i2c.start(addr, 0)
        self.i2c.write(buf)
        if stop:
            self.i2c.stop()

    def writevto(self, addr, vector, stop=True):
        self.i2c.start(addr, 0)
        for data in vector:
            self.i2c.write(data)
        if stop:
            self.i2c.stop()


class Pin:
    OUT = 0
    IN = 1

    def __init__(self, _pin, _dir=0):
        pass

    def on(self):
        pass

    def off(self):
        pass

    @staticmethod
    def value(val=0):
        return val

    def fake(self):
        pass


class SPI(SPIDriver):
    SPIDRIVER_COM_PORT = "/dev/cu.usbserial-DO01HFG1"

    def __init__(self):
        try:
            super().__init__(self.SPIDRIVER_COM_PORT)
            self.unsel()
        except serial.serialutil.SerialException as exc:
            print("###ERR: Failed to connect to SPIDriver controller")
            print(f"###ERR:   {exc}")
            exit()
