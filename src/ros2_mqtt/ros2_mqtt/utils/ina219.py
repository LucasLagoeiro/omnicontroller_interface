import smbus
import time

bus = smbus.SMBus(1)       # I²C bus 1
addr = 0x41                # endereço do INA219



def swap16(val):
    return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF)



while(True):
    raw = bus.read_word_data(addr, 0x02)
    val = swap16(raw)

    print("Valor original:", raw)
    print("Valor ajustado:", val)

    # bits [15..13] = flags, bits [12..3] = VBUS
    vbus_raw = (val >> 3) & 0x1FFF
    volts = vbus_raw * 0.004   # cada LSB = 4 mV

    print(f"Tensão do barramento = {volts:.3f} V")
    time.sleep(1)
    