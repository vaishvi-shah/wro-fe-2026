import time
import board
import adafruit_vl53l0x

i2c = board.I2C()
tof = adafruit_vl53l0x.VL53L0X(i2c)

try:
    while True:
        print(f"Distance: {tof.range} mm")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping ToF test...")
