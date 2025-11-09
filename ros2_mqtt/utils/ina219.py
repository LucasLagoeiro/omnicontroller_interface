import smbus
import time

class INA219:
    """Class to interface with the INA219 sensor via I2C."""
    def __init__(self, address=0x41, busnum=1):
        self._address = address
        self._bus = smbus.SMBus(busnum)

    def read_voltage(self):
        raw = self._bus.read_word_data(self._address, 0x02)
        val = self._swap16(raw) 
        vbus_raw = (val >> 3) & 0x1FFF # bits [15..13] = flags, bits [12..3] = VBUS
        volts = vbus_raw * 0.004   # each LSB = 4 mV
        return volts

    def _swap16(self, val):
        return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF)
