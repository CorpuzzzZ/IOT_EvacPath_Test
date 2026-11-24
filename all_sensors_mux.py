from machine import ADC, Pin, SoftI2C
import struct
import time

VL53 = 0x29
MPU = 0x68
MUX = 0x70
WARMUP_S = 60
RISE_MV = 400      # gas alarm this far above clean air
GYRO_LIMIT = 8     # motion if turning faster than this (deg/s)
ACCEL_LIMIT = 0.08 # motion if acceleration jumps more than this (g)

gas = ADC(Pin(34), atten=ADC.ATTN_11DB)
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=10000)   # slow, for long wires

def read_gas(samples=20):
    total = 0
    for _ in range(samples):
        total += gas.read_uv()
        time.sleep_ms(5)
    return total / samples / 1000 * 2   # undo the half-voltage divider

def select(channel):
    i2c.writeto(MUX, bytes([1 << channel]))

def on_channel(channel, address):
    try:
        select(channel)
        return address in i2c.scan()
    except OSError:
        return False

def read_distance():
    select(0)
    i2c.writeto_mem(VL53, 0x00, b"\x01")
    time.sleep_ms(100)
    for _ in range(50):
        if i2c.readfrom_mem(VL53, 0x14, 1)[0] & 1:
            break
        time.sleep_ms(10)
    else:
        return "no reading"
    d = i2c.readfrom_mem(VL53, 0x14, 12)
    i2c.writeto_mem(VL53, 0x0B, b"\x01")
    status = (d[0] >> 3) & 0x0F
    if status == 8:
        return "too close or covered"
    if status != 11:
        return "no valid reading ({})".format(status)
    return "{} mm".format((d[10] << 8) | d[11])

gyro_zero = [0, 0, 0]

def sample():
    d = i2c.readfrom_mem(MPU, 0x3B, 14)
    ax, ay, az, t, gx, gy, gz = struct.unpack(">hhhhhhh", d)
    return (ax / 8192, ay / 8192, az / 8192), (gx / 65.5, gy / 65.5, gz / 65.5)

def length(v):
    return (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5

def calibrate():
    select(1)
    total = [0, 0, 0]
    for _ in range(30):
        g = sample()[1]
        for k in range(3):
            total[k] += g[k]
        time.sleep_ms(20)
    for k in range(3):
        gyro_zero[k] = total[k] / 30

def read_motion():
    select(1)
    last = None
    turn = jump = 0
    for _ in range(15):
        a, g = sample()
        turn = max(turn, length([g[k] - gyro_zero[k] for k in range(3)]))
        if last:
            jump = max(jump, length([a[k] - last[k] for k in range(3)]))
        last = a
        time.sleep_ms(10)
    moved = turn > GYRO_LIMIT or jump > ACCEL_LIMIT
    return "{} (turn {:.0f} deg/s, X {:.2f} Y {:.2f} Z {:.2f} g)".format(
        "MOTION DETECTED!" if moved else "still", turn, a[0], a[1], a[2])

def safe(read, present):
    if not present:
        return "not found"
    try:
        return read()
    except OSError:
        return "read error"

def main():
    found = i2c.scan()
    print("Found:", [hex(a) for a in found])
    if MUX not in found or len(found) > 5:
        print("Multiplexer not found, check the wiring")
        return
    has_vl53 = on_channel(0, VL53)
    has_mpu = on_channel(1, MPU)
    print("VL53L0X:", has_vl53, "Motion sensor:", has_mpu)
    if has_vl53:
        select(0)
        v = i2c.readfrom_mem(VL53, 0x89, 1)[0]
        i2c.writeto_mem(VL53, 0x89, bytes([v | 1]))
    if has_mpu:
        select(1)
        i2c.writeto_mem(MPU, 0x6B, b"\x00")       # wake up
        time.sleep_ms(100)
        i2c.writeto_mem(MPU, 0x1B, b"\x08")       # +/-500 deg/s
        i2c.writeto_mem(MPU, 0x1C, b"\x08")       # +/-4 g
        time.sleep_ms(100)
        print("Keep it still, calibrating...")
        calibrate()

    start = time.ticks_ms()
    baseline = None
    while True:
        mv = read_gas()
        left = WARMUP_S - time.ticks_diff(time.ticks_ms(), start) // 1000
        if baseline is None and left <= 0:
            baseline = mv
            print("Gas baseline: {:.0f} mV".format(baseline))
        if baseline is None:
            g = "warming up {} s".format(left)
        else:
            g = "{:.0f} mV {}".format(mv, "GAS!" if mv > baseline + RISE_MV else "ok")
        print("Gas:", g, "| Distance:", safe(read_distance, has_vl53),
              "| Motion:", safe(read_motion, has_mpu))
        time.sleep_ms(200)

try:
    main()
except KeyboardInterrupt:
    print("Stopped")