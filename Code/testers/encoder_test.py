import time
import serial
from gpiozero import RotaryEncoder

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing

# BCM pin numbering
PIN_A = 17
PIN_B = 27

# Pulses per one revolution of the output (gearbox) shaft = motor-shaft encoder PPR x gear
# ratio = 11 x 21.3 (620 RPM config) = 234.3, rounded to 234. Assumes RotaryEncoder.steps
# counts one step per PPR click, not x4 quadrature decoding (would be ~936 if so) --
# verify with a manual-rotation count if readings still look off.
PULSES_PER_REV = 234

encoder = RotaryEncoder(PIN_A, PIN_B, max_steps=0)  # max_steps=0 -- no upper bound on counting

DEFAULT_STEER_ANGLE = 100  # neutral/straight steering angle
TEST_SPEED = 70            # forward speed sent to the microcontroller for this test

# CSV protocol: steering,speed,controller,extra,label -- 'test' quoted on the drive
# command, bare on STOP (matches openchallenge_v2.py's own inconsistency)
run_message = f"{DEFAULT_STEER_ANGLE},{TEST_SPEED},FWD,0,'test'\n"
stop_message = f"{DEFAULT_STEER_ANGLE},0,STOP,0,test\n"

# Closed-loop test: target RPM -> measure actual RPM -> error -> adjust speed -> motor ->
# encoder -> repeat. Measures over RPM_SAMPLE_INTERVAL (not every short tick) and rejects
# implausible readings so noise/miscalibration doesn't get amplified into wild speed swings.
TARGET_RPM = 620          # desired output-shaft RPM to hold -- tune on track
KP_RPM = 0.2              # proportional gain -- tune on track
MIN_SPEED = 0             # clamp -- lower bound for the serial `speed` field
MAX_SPEED = 100           # clamp -- upper bound for the serial `speed` field
RPM_SAMPLE_INTERVAL = 0.5  # seconds per measure-and-adjust cycle
MAX_PLAUSIBLE_RPM = 900   # readings above this are treated as bad data, not acted on

speed = TEST_SPEED  # starting point for the control loop; adjusted every cycle below

print(f"Holding TARGET_RPM={TARGET_RPM} via proportional control (KP_RPM={KP_RPM}), "
      f"sampling every {RPM_SAMPLE_INTERVAL}s. Press Ctrl+C to stop.")

last_count = encoder.steps
last_time = time.monotonic()

try:
    while True:
        time.sleep(RPM_SAMPLE_INTERVAL)

        now = time.monotonic()
        dt = now - last_time
        count = encoder.steps
        delta = count - last_count
        rpm = (delta / PULSES_PER_REV) / (dt / 60) if dt > 0 else 0.0
        last_count = count
        last_time = now

        if rpm > MAX_PLAUSIBLE_RPM:
            print(f"actual={rpm:.1f} -- implausible (> {MAX_PLAUSIBLE_RPM}), skipping this "
                  f"reading, speed unchanged at {speed:.0f}")
            continue

        error = TARGET_RPM - rpm
        speed += KP_RPM * error
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))

        ser.write(f"{DEFAULT_STEER_ANGLE},{speed:.0f},FWD,0,'test'\n".encode())
        ser.flush()

        print(f"target={TARGET_RPM} actual={rpm:.1f} error={error:.1f} speed={speed:.0f}")

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    ser.write(stop_message.encode())
    ser.flush()
    ser.close()
    encoder.close()
    print("\nStopped")
