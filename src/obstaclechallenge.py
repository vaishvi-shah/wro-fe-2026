
# imports!
import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055

CALIBRATION_FILE = "src/lib/bno055_calibration.json"
bno055.initialize()            # boot IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present
controller = 1
sent_steer = 0
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent as 100 + this
LINE_COUNT = 12                  # number of colour-line crossings before stopping
SAFE_TURN_AREA = 2000           # max black area on side of a turn before safe to execute
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)

# obstacle-avoidance tuning
COLLISION_BLACK_AREA = 4000     # black area in middle frame counted as wall collision
COLLISION_CLEAR_AREA = 1500     # black area must drop below this before reverse ends
OBSTACLE_AREA = 1200            # red/green area big enough to react to
AVOID_KP = 0.02                 # proportional gain for red/green avoidance steer
REVERSE_SPEED = -400            # speed while backing away from a collision
AVOID_SPEED = 400                # speed while steering around an obstacle

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to steering/speed microcontroller
time.sleep(2)  # let serial connection settle before writing
steering = 100 + DEFAULT_STEER_ANGLE
stop = False
speed = 0
pending_turn = False  # true if turn pending (colour line seen, not yet executed)

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour seen ("CWR" or "CCL")

frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()

turning = False           # true while inside post-detection "turn window"
turning_time = time.time()  # timestamp of last colour-line detection

colliding = False       # true while backing away from a black-wall collision in middle frame
avoiding = False        # true while steering around a red/green obstacle in middle frame
avoid_direction = ''    # side to steer toward while avoiding/backing ("CWR" or "CCL")

# defining colour ranges (HSV) used to mask each region of interest
black_range = [[np.array([0, 0, 0]), np.array([180, 200, 60])]]

blue_range = [[np.array([105, 140, 80]), np.array([135, 255, 255])]]

orange_range = [[np.array([10, 60, 100]), np.array([25, 255, 255])]]

red1_range = [[np.array([0, 80, 40]), np.array([10, 255, 255])]]
red2_range = [[np.array([170, 80, 40]), np.array([180, 255, 255])]]

green_range = [[np.array([40, 70, 40]), np.array([85, 255, 255])]]

# Function to navigate straight along wall based on number of black pixels on either wall
# if more black on a wall, turn steering other way proportional to difference of black pixels

desired_heading = bno055.get_heading()

# One-off startup check: confirm gyro readable and report calibration state before main loop.
print("Reading gyro heading. Press Ctrl+C to exit.")
cal_status = sensor.calibration_status
heading = bno055.get_heading()
if heading is not None:
    print(f"Heading: {heading:7.2f} | Cal Status (S,G,A,M): {cal_status}")
else:
    print("Could not read heading.")
time.sleep(0.1)

# Blend weights: 70% gyro heading hold, 30% camera wall-pixel balance.
GYRO_WEIGHT = 0.7
CAM_WEIGHT = 0.3


def turn(direction):
    global desired_heading

    if direction == "CWR":      # clockwise = right turn
        desired_heading += 90
        print("CWR")
    elif direction == "CCL":   # counterclockwise = left turn
        desired_heading -= 90
        print("CCL")

    # normalize to 0-360 range
    desired_heading = desired_heading % 360


def heading_to_signed(heading):
    """
    Converts raw gyro heading (0-359 deg, where left turns increase
    normally and right turns wrap around through 360) into signed
    angle: 0 = straight, positive = left, negative = right.

    Examples: 0->0, 20->20, 45->45, 180->180, 359->-1, 350->-10, 325->-35
    """
    if heading <= 180:
        return heading
    return heading - 360


def angle_error(current, target):
    error = (current - target + 180) % 360 - 180
    return error


def navigate_wall(gyro_heading, desired_heading=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro_heading vs desired_heading).
      2. Camera term: proportional correction on left/right wall pixel area difference.
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [30, 150].
    """
    # Refresh side frames with latest camera capture and re-run
    # colour mask + contour detection so we know how much "wall" each side sees.
    left_frame.update(cap)
    right_frame.update(cap)

    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()

    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    # Camera term: more black pixels on one side pushes steering
    # away from that side, proportional to area gap.
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)

    # Gyro term: drives heading error toward zero. Falls back to straight if gyro unavailable.
    if gyro_heading is not None:
        signed_heading = heading_to_signed(gyro_heading)  # convert raw 0-359 reading to signed angle
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE

    # Weighted blend of two independent steering estimates.
    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer
    steering_value = max(30, min(150, steering_value))  # clamp to servo range

    print(f"gyro heading: {gyro_heading:.0f}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, steer: {steering_value:.0f}")

    return int(steering_value)


# execution of main program
cv2.startWindowThread()  # needed so cv2.imshow updates without blocking on this thread

# initializing camera
print("-- INITIALIZING CAMERA --")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()
cap = picam2.capture_array("main")  # grab one frame to size ROI frames below

# initializing frames: each Frame watches a fixed region of interest (ROI) for a colour mask.
# left/right strips watch black wall; bottom strip watches blue/orange turn markers;
# middle frame watches red/green obstacle markers and black for collision detection.
left_frame = Frame(cap, 0, 20, 60, 200, colour_range=black_range)
right_frame = Frame(cap, 300, 320, 60, 200, colour_range=black_range)
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])
middle_frame = Frame(cap, 220, 420, 140, 340, colour_range=[red1_range, red2_range,green_range, black_range])

print("ENTERING THE WHILE LOOP")

while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable

    # --- Priority 1: middle-frame collision (black wall directly ahead) ---
    middle_frame.update(cap)
    red_contours, green_contours, mid_black_contours = middle_frame.find_contours(
        colour=(0, 0, 255), colour2=(0, 255, 0), colour3=(50, 50, 50)
    )
    middle_area, middle_colour = middle_frame.get_areas(red_contours, green_contours, mid_black_contours)
    # middle_colour: 1 = red, 2 = green, 3 = black

    if middle_colour == 3 and middle_area > COLLISION_BLACK_AREA:
        colliding = True
    elif colliding and not (middle_colour == 3 and middle_area > COLLISION_CLEAR_AREA):
        colliding = False

    # --- Priority 2: middle-frame obstacle (red/green), only tracked while not colliding ---
    if middle_colour in (1, 2) and middle_area > OBSTACLE_AREA:
        avoiding = True
        avoid_direction = "CWR" if middle_colour == 1 else "CCL"
    elif avoiding and not (middle_colour in (1, 2) and middle_area > OBSTACLE_AREA):
        avoiding = False

    if colliding:
        # back away from wall, steering toward last known avoid side to clear obstacle
        speed = REVERSE_SPEED
        if avoid_direction == "CWR":
            steering = 100 + 150
        elif avoid_direction == "CCL":
            steering = 100 + 30
        else:
            steering = 100 + DEFAULT_STEER_ANGLE
        print(f"COLLISION: reversing, steer {steering}")

    elif avoiding:
        # steer around obstacle: red -> clockwise/right, green -> counterclockwise/left
        speed = AVOID_SPEED
        offset = AVOID_KP * middle_area
        if avoid_direction == "CWR":
            steer_val = DEFAULT_STEER_ANGLE + offset
        else:
            steer_val = DEFAULT_STEER_ANGLE - offset
        steer_val = max(30, min(150, steer_val))
        steering = 100 + int(steer_val)
        print(f"AVOIDING {avoid_direction}: steer {steering}")

    else:
        # --- Priority 3: bottom-frame turn line ---
        if not turning:
            bottom_frame.update(cap)
            blue_contours, orange_contours = bottom_frame.find_contours(colour=(255, 255, 0), colour2=(0, 127, 255))
            bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours)  # 1 = blue, 2 = orange

            if bottom_area > 800:
                if not direction:
                    if bottom_colour == 1:
                        direction = "CCL"   # blue seen first -> locked to blue/left
                    elif bottom_colour == 2:
                        direction = "CWR"   # orange seen first -> locked to orange/right

                if (direction == "CCL" and bottom_colour == 1) or (direction == "CWR" and bottom_colour == 2):
                    turning_time = time.time()
                    turning = True

                    if bottom_colour == 1:
                        blue_count += 1
                        print(f"BLUE: {blue_count}")
                    elif bottom_colour == 2:
                        orange_count += 1
                        print(f"ORANGE: {orange_count}")

                    pending_turn = True
        else:
            if time.time() - turning_time > 1.5:
                turning = False

        # --- Priority 4: straight/wall-follow (gyro + camera blend) ---
        steering = 100 + navigate_wall(gyro, desired_heading)
        speed = 500

    if pending_turn:  # if turn pending, execute it and reset flag
        left_frame.update(cap)
        right_frame.update(cap)

        left_contours = left_frame.find_contours()
        right_contours = right_frame.find_contours()

        left_area, _ = left_frame.get_areas(left_contours)
        right_area, _ = right_frame.get_areas(right_contours)

        # check safe to turn (not too close to wall)
        if direction == "CCL":  # left
            if left_area < SAFE_TURN_AREA:
                turn(direction)
                pending_turn = False

        elif direction == "CWR":  # right
            if right_area < SAFE_TURN_AREA:
                turn(direction)
                pending_turn = False

    # Once either colour crossed LINE_COUNT times, start stop sequence.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True

    if stop:
        if (time.time() - stop_time > 1):  # short grace period before actually stopping
            print("Stopping")
            speed = 0000
            ser.write(f"19020000\n".encode())  # send fixed stop command
            ser.flush()
            break

    if (SHOW_VID):
        # Overlay debug info (steering angle, line counts, FPS) on preview frame.
        cv2.putText(cap, f"Steer: {steering:.2f}", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(cap, f"O: {str(orange_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 255), 1)
        cv2.putText(cap, f"B: {str(blue_count)}", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 127, 0), 1)

        frame_count += 1
        elapsed = time.time() - frame_time
        if elapsed > 1.0:
            fps = frame_count // elapsed  # rough FPS sampled once per second
            frame_count = 0

            frame_time = time.time()
        cv2.putText(cap, f"FPS: {fps:.2f}", (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Video Frame", cap)

    if abs(sent_steer - steering) >= 3:
        ser.write(f"{steering:.0f}{speed+1000}{controller}\n".encode())  # send steering+speed to microcontroller
        sent_steer = steering
        ser.flush()
    time.sleep(0.01)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends stop command
        ser.write(f"19010230\n".encode())
        ser.flush()
        break
ser.close()

cv2.destroyAllWindows()