"""
Single-file RPM-controlled DC motor driver for a Raspberry Pi + L298N.

Hardware:

    L298N pin      Raspberry Pi GPIO (BCM numbering)
    ---------      ---------------------------------
    IN1        ->  GPIO 5      (direction bit 1)
    IN2        ->  GPIO 6      (direction bit 2)
    ENA        ->  GPIO 13     (PWM speed control)

    Encoder A  ->  GPIO 17
    Encoder B  ->  GPIO 27
"""

import time
from gpiozero import RotaryEncoder, PWMOutputDevice, DigitalOutputDevice


# ============================================================
# PIN CONFIGURATION
# ============================================================

# --- Motor driver (L298N) ---
IN1_PIN = 5
IN2_PIN = 6
ENA_PIN = 13
PWM_FREQUENCY = 1000  # Hz

# --- Encoder ---
ENCODER_A_PIN = 17
ENCODER_B_PIN = 27


# ============================================================
# DRIVETRAIN / ENCODER CONSTANTS
# ============================================================

GEARBOX_RATIO = 9.6
DIFFERENTIAL_RATIO = 2.5
ENCODER_PPR = 11        # Hall pulses per motor revolution (datasheet)
QUADRATURE = 4          # gpiozero RotaryEncoder counts x4 per cycle

# Theoretical steps per WHEEL revolution -- CALIBRATE THIS
#STEPS_PER_WHEEL_REV = ENCODER_PPR * QUADRATURE * GEARBOX_RATIO * DIFFERENTIAL_RATIO
# = 1056
STEPS_PER_WHEEL_REV = 200


# Reference only, not enforced anywhere below.
GEARBOX_OUTPUT_RPM_NO_LOAD = 620
MAX_WHEEL_RPM_NO_LOAD = GEARBOX_OUTPUT_RPM_NO_LOAD / DIFFERENTIAL_RATIO
# ~= 248 RPM no-load; expect less once the robot's weight/friction loads the motor.

# ============================================================
# CONTROL LOOP SETTINGS
# ============================================================

CONTROL_PERIOD = 0.05  # seconds between controller updates

# Uncomment / tune once you've measured your motor's dead zone:
# MIN_RUNNING_PWM = 70
MIN_RUNNING_PWM = 0

# ============================================================
# ENCODER / RPM MEASUREMENT
# ============================================================

class EncoderRPM:
    def __init__(self, pin_a, pin_b, steps_per_rev):
        self.encoder = RotaryEncoder(pin_a, pin_b, max_steps=0)
        self.steps_per_rev = steps_per_rev

        self.last_count = self.encoder.steps
        self.last_time = time.monotonic()
        self.rpm = 0.0

    def update(self):
        now = time.monotonic()
        count = self.encoder.steps

        dt = now - self.last_time
        delta = count - self.last_count

        self.last_count = count
        self.last_time = now

        if dt <= 0:
            return self.rpm

        revolutions = delta / self.steps_per_rev
        raw_rpm = revolutions / dt * 60.0
        self.rpm = abs(raw_rpm)
        return self.rpm

    def get_count(self):
        return self.encoder.steps

    def reset(self):
        self.last_count = self.encoder.steps
        self.last_time = time.monotonic()
        self.rpm = 0.0

    def close(self):
        self.encoder.close()


# ============================================================
# PID SPEED CONTROLLER  (operates in 0-255 "virtual PWM" units)
# ============================================================

class RPMController:
    """
    output = kp*error + ki*integral(error) + kd*d(error)/dt

    Tune kp, then ki, then kd -- one gain at a time.
    """

    def __init__(self, kp, ki, kd, min_pwm, max_pwm):
        self.kp = kp   # main push term, proportional to current error
        self.ki = ki   # closes steady-state error from friction/load
        self.kd = kd   # damps overshoot; noisy at low RPM, tune last

        self.min_pwm = min_pwm
        self.max_pwm = max_pwm

        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

    def update(self, target_rpm, current_rpm, dt):
        if target_rpm <= 0:
            self.reset()
            return 0

        error = target_rpm - current_rpm

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # only commit this cycle's integral if it doesn't push output past the PWM limits
        # (anti-windup: freezing the integral on saturation avoids overshoot once it clears)
        tentative_integral = self.integral + error * dt
        raw_output = (
            self.kp * error
            + self.ki * tentative_integral
            + self.kd * derivative
        )

        if self.min_pwm <= raw_output <= self.max_pwm:
            self.integral = tentative_integral

        self.output = max(self.min_pwm, min(self.max_pwm, raw_output))

        return int(round(self.output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

# ============================================================
# MOTOR DRIVER  (direct L298N control, no Arduino / no serial)
# ============================================================

class MotorDriver:
    def __init__(self, in1_pin, in2_pin, ena_pin, pwm_frequency=1000):
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)
        self.ena = PWMOutputDevice(ena_pin, frequency=pwm_frequency)

    def drive(self, pwm_0_255, direction):
        """direction: "FWD", "BWD", or "STOP" """
        pwm_0_255 = max(0, min(255, int(pwm_0_255)))
        duty = pwm_0_255 / 255.0

        if direction == "FWD" and pwm_0_255 > 0:
            self.in2.on()
            self.in1.off()
            self.ena.value = duty
        elif direction == "BWD" and pwm_0_255 > 0:
            self.in2.off()
            self.in1.on()
            self.ena.value = duty
        else:
            self.stop()

    def stop(self):
        self.ena.value = 0
        self.in1.off()
        self.in2.off()

    def close(self):
        self.stop()
        self.in1.close()
        self.in2.close()
        self.ena.close()


# ============================================================
# SHARED OBJECTS  (one motor, one encoder, one controller)
# ============================================================

encoder = EncoderRPM(ENCODER_A_PIN, ENCODER_B_PIN, STEPS_PER_WHEEL_REV)
controller = RPMController(kp=1.5, ki=20, kd=0.0, min_pwm=0, max_pwm=255)
motor = MotorDriver(IN1_PIN, IN2_PIN, ENA_PIN, PWM_FREQUENCY)


# ============================================================
# maintainRPM -- the main thing you call from your program
# ============================================================

# def maintainRPM(rpm, duration=0):
#     """
#     Drive to and hold a target signed wheel RPM (+forward, -backward, 0 stops immediately).
#     duration=0 runs forever (until Ctrl+C); duration>0 holds for that many seconds then
#     returns without stopping the motor -- call stopMotor() after if you want it to stop.
#     """

#     print("Entering Maintain RPM")

#     if rpm == 0:
#         stopMotor()
#         return

#     direction = "forward" if rpm > 0 else "backward"
#     target_rpm = abs(rpm)

#     controller.reset()
#     encoder.reset()

#     start_time = time.monotonic()
#     last_control_time = start_time

#     for i in range(100):
#         # print("Looping ", i)
#         now = time.monotonic()

#         if duration > 0 and (now - start_time) >= duration:
#             break

#         dt = now - last_control_time

#         if dt >= CONTROL_PERIOD:
#             last_control_time = now

#             current_rpm = encoder.update()
#             pwm = controller.update(target_rpm, current_rpm, dt)

#             if 0 < pwm < MIN_RUNNING_PWM:
#                 pwm = MIN_RUNNING_PWM

#             motor.drive(pwm, direction)

#             print(
#                 f"Target: {target_rpm:5.1f} RPM | "
#                 f"Actual: {current_rpm:5.1f} RPM | "
#                 f"PWM: {pwm:3d} | "
#                 f"Dir: {direction:>8s} | "
#                 f"Encoder: {encoder.get_count()}"
#             )

#         time.sleep(0.005)


def stopMotor():
    """Immediately stop the motor and clear the controller's state."""
    motor.stop()
    controller.reset()


def driveRotations(rotations, pwm, direction):
    """
    Drive the motor at a fixed PWM until the encoder reports the given
    number of wheel rotations have been completed, then stop.

    rotations: number of wheel rotations to travel (int, >0)
    pwm: drive speed, 0-255
    direction: "FWD" or "BWD"
    """

    print("Entering Drive Rotations")

    if direction not in ['FWD','BWD']:
        print ("Invalid Direction Value")
        return 

    if rotations <= 0:
        motor.stop()
        return

    target_steps = rotations * STEPS_PER_WHEEL_REV

    encoder.reset()
    start_count = encoder.get_count()

    motor.drive(pwm, direction)

    while abs(encoder.get_count() - start_count) < target_steps:
        
        time.sleep(0.005)

    motor.stop()


def driveMotor(pwm, direction):
    """
    Drive the motor at a fixed PWM indefinitely -- runs until stopMotor()
    is called (this function returns immediately, it does not block).

    pwm: drive speed, 0-255
    direction: "FWD" or "BWD"
    """

    print("Entering Drive Motor")

    if direction not in ['FWD', 'BWD']:
        print("Invalid Direction Value")
        return

    motor.drive(pwm, direction)


# ============================================================
# MAIN  (example usage -- replace with your own sequence)
# ============================================================

def main():
    print("===================================")
    print("  RPM MOTOR CONTROLLER (direct GPIO, no Arduino)")
    print("===================================")
    print("Press CTRL+C at any time to stop.\n")

    try:
        #driveRotations(2,80,'FWD')
        print("Here")
        driveMotor(254,'FWD')
        time.sleep(5)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        stopMotor()
        motor.close()
        encoder.close()
        print("Motor stopped.")


if __name__ == "__main__":
    main()
