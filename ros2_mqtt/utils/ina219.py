import smbus
import time

class INA219:
    """Classe para interfacear o sensor INA219 via barramento I2C."""

    def __init__(self, address=0x41, busnum=1):
        # Endereço I2C e número do barramento (ex: /dev/i2c-1)
        self._address = address
        self._bus = smbus.SMBus(busnum)

    def read_voltage(self):
        """
        Lê a tensão do barramento VBUS (registrador 0x02)
        Cada unidade LSB equivale a 4 mV.
        """
        raw = self._bus.read_word_data(self._address, 0x02)  # Lê palavra de 16 bits
        val = self._swap16(raw)  # Corrige a ordem dos bytes 
        vbus_raw = (val >> 3) & 0x1FFF  # Extrai bits 12..3 correspondentes à tensão
        volts = vbus_raw * 0.004  # Converte para volts
        return volts

    def _swap16(self, val):
        """Inverte a ordem dos bytes."""
        return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF)
