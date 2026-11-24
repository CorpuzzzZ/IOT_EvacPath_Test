from machine import ADC, Pin
import time

RISE_MV = 400      # alarm this far above clean air; tune it
adc = ADC(Pin(34), atten=ADC.ATTN_11DB)

def read_mv(samples=20):
    total = 0
    for _ in range(samples):
        total += adc.read_uv()
        time.sleep_ms(5)
    return total / samples / 1000 * 2   # undo the half-voltage divider

try:
    print("Warming up 60 s in clean air...")
    time.sleep(60)
    baseline = read_mv()
    print("Baseline: {:.0f} mV".format(baseline))
    while True:
        mv = read_mv()
        status = "GAS!" if mv > baseline + RISE_MV else "ok"
        print("Gas: {:.0f} mV  {}".format(mv, status))
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped")