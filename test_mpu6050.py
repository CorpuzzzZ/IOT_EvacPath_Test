from machine import SoftI2C, Pin
import time
import struct

ADDR = 0x68
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=10000)   # slow, for long wires

def read_all():
    d = i2c.readfrom_mem(ADDR, 0x3B, 14)
    ax, ay, az, t, gx, gy, gz = struct.unpack(">hhhhhhh", d)
    accel = [v / 16384 for v in (ax, ay, az)]     # g
    gyro = [v / 131 for v in (gx, gy, gz)]        # degrees per second
    return accel, gyro, t / 340 + 36.53           # temperature in C

def main():
    found = i2c.scan()
    print("Found:", [hex(a) for a in found])
    if ADDR not in found or len(found) > 5:
        print("MPU-6050 not found, check the wiring")
        return
    print("ID:", hex(i2c.readfrom_mem(ADDR, 0x75, 1)[0]))
    i2c.writeto_mem(ADDR, 0x6B, b"\x00")          # wake up
    time.sleep_ms(100)
    i2c.writeto_mem(ADDR, 0x1B, b"\x00")          # gyro +/-250 deg/s
    i2c.writeto_mem(ADDR, 0x1C, b"\x00")          # accel +/-2 g
    time.sleep_ms(100)
    print("MPU-6050 OK")
    while True:
        a, g, t = read_all()
        total = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
        print("Accel X {:.2f} Y {:.2f} Z {:.2f} (total {:.2f} g) | Gyro X {:.0f} Y {:.0f} Z {:.0f} | {:.1f} C".format(
            a[0], a[1], a[2], total, g[0], g[1], g[2], t))
        time.sleep_ms(500)

try:
    main()
except KeyboardInterrupt:
    print("Stopped")