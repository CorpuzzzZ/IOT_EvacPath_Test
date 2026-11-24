from machine import SoftI2C, Pin
import time

ADDR = 0x29
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=10000)   # slow, for long wires

def read_mm():
    i2c.writeto_mem(ADDR, 0x00, b"\x01")          # start one measurement
    time.sleep_ms(100)
    for _ in range(50):
        if i2c.readfrom_mem(ADDR, 0x14, 1)[0] & 1:
            break
        time.sleep_ms(10)
    else:
        return None, -1
    d = i2c.readfrom_mem(ADDR, 0x14, 12)
    i2c.writeto_mem(ADDR, 0x0B, b"\x01")          # clear for the next one
    return (d[10] << 8) | d[11], (d[0] >> 3) & 0x0F

def main():
    found = i2c.scan()
    print("Found:", [hex(a) for a in found])
    if found != [ADDR]:
        print("Expected only 0x29, check the wiring")
        return
    chip = i2c.readfrom_mem(ADDR, 0xC0, 1)[0]
    if chip != 0xEE:
        print("Unexpected ID:", hex(chip))
        return
    print("VL53L0X OK")
    v = i2c.readfrom_mem(ADDR, 0x89, 1)[0]
    i2c.writeto_mem(ADDR, 0x89, bytes([v | 1]))   # 2.8 V I/O mode
    while True:
        mm, status = read_mm()
        if status == 11:
            print("Distance: {} mm ({:.1f} cm)".format(mm, mm / 10))
        else:
            print("No valid reading, raw:", mm, "status:", status)
        time.sleep_ms(400)

try:
    main()
except KeyboardInterrupt:
    print("Stopped")