"""
JGA25-371 Motor + Encoder Test
Raspberry Pi + L293D
(ported from the Arduino Nano version)

Wiring (BCM numbering):
    ENC_A       -> GPIO17 (physical pin 11)
    ENC_B       -> GPIO27 (physical pin 13)
    MOTOR_IN1   -> GPIO22 (physical pin 15)
    MOTOR_IN2   -> GPIO23 (physical pin 16)
    MOTOR_EN    -> GPIO18 (physical pin 12, hardware PWM)

Requires: pip install gpiozero lgpio
(gpiozero auto-selects the lgpio pin factory on the Pi 5's RP1 chip)
"""

import time
import threading
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice

ENC_A = 17
ENC_B = 27
MOTOR_IN1 = 22
MOTOR_IN2 = 23
MOTOR_EN = 18

encoder_count = 0
count_lock = threading.Lock()

enc_a = DigitalInputDevice(ENC_A, pull_up=True)
enc_b = DigitalInputDevice(ENC_B, pull_up=True)

motor_in1 = DigitalOutputDevice(MOTOR_IN1)
motor_in2 = DigitalOutputDevice(MOTOR_IN2)
motor_en = PWMOutputDevice(MOTOR_EN, frequency=1000)


def encoder_isr():
    global encoder_count
    with count_lock:
        if enc_a.value == enc_b.value:
            encoder_count += 1
        else:
            encoder_count -= 1


# Equivalent of attachInterrupt(ENC_A, encoderA, CHANGE)
enc_a.when_activated = encoder_isr
enc_a.when_deactivated = encoder_isr

# Motor direction: FORWARD
motor_in1.on()
motor_in2.off()

# Motor speed: 50% (analogWrite(128) on a 0-255 scale -> 0.0-1.0 scale)
motor_en.value = 128 / 255

print("Encoder counter started")
print("Motor running")

try:
    while True:
        time.sleep(0.5)
        with count_lock:
            count = encoder_count
        print(f"Encoder count: {count}")
except KeyboardInterrupt:
    pass
finally:
    motor_en.value = 0
    motor_in1.off()
    motor_in2.off()
    print("\nStopped")
