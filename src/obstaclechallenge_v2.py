# 1 ----------------------------------------------------------------------------------------

# CHANGE DESIRED HEADING VERSION -----

'''
#1 — Hybrid weighted control (gyro + camera as two separate steering inputs)
Camera and gyro each compute their own steering value independently:
cam_steer (wall imbalance → direct correction)
gyro_steer (heading error → direct correction)
Final steering is a blend:
70% gyro + 30% camera
Meaning:
Camera directly fights steering every loop.
Gyro also directly fights steering every loop.
Behavior:
More reactive to sensor noise.
Two controllers "compete" and are averaged.


Core idea:


"Both sensors directly output steering, then we mix them."


'''


# imports!
import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055


CALIBRATION_FILE = "lib/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present
controller = 1
sent_steer = 0
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent as 100 + this
LINE_COUNT = 12                # number of colour-line crossings before stopping
SAFE_TURN_AREA = 2000           # max black area on the side of a turn before we can safely execute the turn
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)
WALL_OFFSET_AVOIDING = 1000  # extra black area added to the side of a red/green obstacle to bias steering away from it (unused now, kept for reference)
OBSTACLE_ENTER_AREA = 400    # min red/green contour area (px) to start an avoidance maneuver (filters out noise)
OBSTACLE_EXIT_AREA = 150     # area threshold below which the sign is considered "cleared" (lower than ENTER = hysteresis)
OBSTACLE_CLEAR_FRAMES = 5    # consecutive frames below OBSTACLE_EXIT_AREA required before ending avoidance
OBSTACLE_MAX_AVOID_TIME = 3.0  # failsafe only: hard cap in case the sign never visually clears

# --- Obstacle offset controller (Option 3: steer using the obstacle's own pixel
# position in middle_frame, rather than injecting fake wall area) ---
KP_OBSTACLE = 0.15   # proportional gain on obstacle pixel error -> tune on the bot
KD_OBSTACLE = 0.02   # reserved for a derivative term if the bias needs smoothing (unused for now)
# middle_frame spans x=25..295 -> ROI width 270px, ROI-local x runs 0..270, center ~135.
# TARGET_OFFSET_RED/GREEN are ROI-local pixel targets: where we want the obstacle's
# centroid to sit once we're steering around it (re-check/tune these against your ROI).
TARGET_OFFSET_RED = 220     # red -> keep it toward the right of the ROI, so we pass on its left
TARGET_OFFSET_GREEN = 50    # green -> keep it toward the left of the ROI, so we pass on its right

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = 100 + DEFAULT_STEER_ANGLE
stop = False
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)


blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")
avoiding = False      # true while avoiding a red/green obstacle
avoiding_time = time.time()  # timestamp of the start of the current avoidance window (used only as a failsafe cap)
avoiding_colour = None  # colour (1=red, 2=green) that triggered the current avoidance window
clear_count = 0      # consecutive frames the obstacle's area has been below OBSTACLE_EXIT_AREA
obstacle_steer_bias = 0  # steering bias (added directly to steering_value) from the obstacle offset controller

frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()


turning = False           # true while inside the post-detection "turn window"
turning_time = time.time()  # timestamp of the last colour-line detection


# defining colour ranges (HSV) used to mask each region of interest
blue_range = [
    [np.array([105, 140, 80]), np.array([135, 255, 255])]
]

orange_range = [
    [np.array([10, 60, 100]), np.array([25, 255, 255])]
]

black_range = [
    [np.array([0, 0, 0]), np.array([180, 200, 60])]
]

# Red wraps around the 0/180 hue boundary in OpenCV's HSV space, so it needs
# two ranges (low end + high end) grouped together as one colour.
red1_range = [
    [np.array([0, 80, 40]), np.array([10, 255, 255])]
]
red2_range = [
    [np.array([170, 80, 40]), np.array([180, 255, 255])]
]
red_range = red1_range + red2_range   # one colour group, two HSV ranges (hue wraps at 0/180)

green_range = [
    [np.array([40, 70, 40]), np.array([85, 255, 255])]
]

# Function to navigate straight along the wall based on the number of black pixels on either wall
# if more black on a wall, turn steering the other way proportional to difference of black pixels


desired_heading = bno055.get_heading()


# One-off startup check: confirm gyro is readable and report calibration state before the main loop.
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


    # normalize to 0–360 range
    desired_heading = desired_heading % 360


def heading_to_signed(heading):
    """
    Converts a raw gyro heading (0-359 deg, where left turns increase
    normally and right turns wrap around through 360) into a signed
    angle: 0 = straight, positive = left, negative = right.


    Examples: 0->0, 20->20, 45->45, 180->180, 359->-1, 350->-10, 325->-35
    """
    if heading <= 180:
        return heading
    return heading - 360


def angle_error(current, target):
    error = (current - target + 180) % 360 - 180
    return error    

def contour_area(contours):
    """
    Total pixel area of a list of contours (0 if none). Used to measure how
    much of a red/green sign is currently visible, so the avoidance maneuver
    can be driven by what the camera actually sees rather than a fixed timer.
    """
    if not contours:
        return 0
    return sum(cv2.contourArea(c) for c in contours)

def contour_centroid_x(contours):
    """
    Returns the ROI-local x-pixel centroid of the largest contour in
    `contours`, or None if there's nothing to measure. Used to find where
    a red/green obstacle currently sits horizontally in middle_frame, so
    the obstacle offset controller can steer toward a target offset
    instead of just reacting to how big the sign is.
    """
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"]

def navigate_wall(gyro_heading, desired_heading=0, KP=0.05, KP_GYRO=0.5, left_area=0, right_area=0, obstacle_bias=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro_heading vs desired_heading).
      2. Camera term: proportional correction on left/right wall pixel area difference (original logic).
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [30, 150].
    """


    # Camera term (unchanged from original): more black pixels on one side
    # pushes steering away from that side, proportional to the area gap.
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)


    # Gyro term: drives heading error toward zero. Falls back to straight if gyro unavailable.
    if gyro_heading is not None:
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE


    # Weighted blend of the two independent steering estimates, plus the
    # obstacle offset bias (0 unless we're actively avoiding a red/green sign).
    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer + obstacle_bias

    # Wall safety clamp: don't let the obstacle bias steer us into a wall that's
    # already close. If the left wall area is high, biasing further left
    # (obstacle_bias < 0) gets capped; same idea on the right.
    if left_area > SAFE_TURN_AREA and obstacle_bias < 0:
        steering_value = max(steering_value, DEFAULT_STEER_ANGLE - 10)
    if right_area > SAFE_TURN_AREA and obstacle_bias > 0:
        steering_value = min(steering_value, DEFAULT_STEER_ANGLE + 10)

    steering_value = max(30, min(150, steering_value))  # clamp to servo range


    print(f"gyro heading: {gyro_heading:.0f}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, obs bias: {obstacle_bias:.1f}, steer: {steering_value:.0f}")


    return int(steering_value)


# execution of main program
cv2.startWindowThread()  # needed so cv2.imshow updates without blocking on this thread


# initializing the camera
print("-- INITIALIZING CAMERA --")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()
cap = picam2.capture_array("main")  # grab one frame to size the ROI frames below


# initializing frames: each Frame watches a fixed region of interest (ROI) for a colour mask.
# left/right strips watch for the black wall; bottom strip watches for blue/orange turn markers.
left_frame = Frame(cap, 0, 20, 60, 200, colour_range=[black_range])
right_frame = Frame(cap, 300, 320, 60, 200, colour_range=[black_range])
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])

# Middle ROI: spans almost the full width (just inside the left/right wall
# strips at x=20 and x=300), sits just above the bottom turn-marker strip
# (which starts at y=200), and runs tall without covering the full frame
# height (starts at y=20 rather than y=0). Watches for red, green, and black.
middle_frame = Frame(cap, 25, 295, 20, 195, colour_range=[red_range, green_range, black_range])

print("ENTERING THE WHILE LOOP")


while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable
    speed = 800


    # CHECKING FOR OBSTACLES -------------------

    middle_frame.update(cap)
    mid_red_contours, mid_green_contours, mid_black_contours = middle_frame.find_contours()

    # Measure red and green area directly, independent of black. (The old
    # code picked a single "biggest of the three" winner, which meant a
    # visible sign could be masked out by the wall/track being bigger in the
    # same ROI — we want the sign's own size, not how it compares to black.)
    red_area = contour_area(mid_red_contours)
    green_area = contour_area(mid_green_contours)

    if red_area > 0 or green_area > 0:
        cv2.putText(
            cap,
            f"Mid: R({red_area:.0f}) G({green_area:.0f})",
            (90, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

    # Start avoiding as soon as a red/green contour is big enough to be a
    # real sign rather than noise. If both appear at once, react to
    # whichever is currently larger.
    if not avoiding:
        if red_area > OBSTACLE_ENTER_AREA or green_area > OBSTACLE_ENTER_AREA:
            avoiding = True
            avoiding_time = time.time()
            clear_count = 0
            avoiding_colour = 1 if red_area >= green_area else 2


    # GETTING STEERING CALCULATION -------------------

    # Refresh the side frames with the latest camera capture and re-run the
    # colour mask + contour detection so we know how much "wall" each side sees.
    
    
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    # IF AVOIDING, BIAS STEERING USING THE OBSTACLE'S OWN PIXEL POSITION UNTIL
    # THE CAMERA CONFIRMS IT HAS ACTUALLY BEEN PASSED, rather than a fixed timer.
    # obstacle_steer_bias is computed from how far the obstacle's centroid
    # (obs_x, in middle_frame's ROI-local x coords) is from a target offset,
    # and is added directly onto navigate_wall's steering_value; the
    # left/right wall-following term keeps running unmodified in parallel as
    # a safety net, and navigate_wall clamps the bias if it'd push us into a
    # wall that's already close (see the SAFE_TURN_AREA check there).
    #
    # Exit condition: the avoided colour's own contour area has to drop below
    # OBSTACLE_EXIT_AREA (lower than OBSTACLE_ENTER_AREA -> hysteresis, so it
    # doesn't flicker in/out right at the boundary) and stay there for
    # OBSTACLE_CLEAR_FRAMES consecutive frames (so one missed detection
    # doesn't end the maneuver early). OBSTACLE_MAX_AVOID_TIME is only a
    # failsafe in case the sign never visually clears.

    if avoiding:
        current_area = red_area if avoiding_colour == 1 else green_area
        contours = mid_red_contours if avoiding_colour == 1 else mid_green_contours
        obs_x = contour_centroid_x(contours)

        if current_area > OBSTACLE_EXIT_AREA:
            clear_count = 0   # still clearly see it -> reset the "gone" counter
        else:
            clear_count += 1  # shrunk below the exit threshold this frame

        timed_out = time.time() - avoiding_time > OBSTACLE_MAX_AVOID_TIME

        if clear_count >= OBSTACLE_CLEAR_FRAMES or timed_out:
            avoiding = False
            avoiding_colour = None
            clear_count = 0
            obstacle_steer_bias = 0
        else:
            if obs_x is not None:
                target = TARGET_OFFSET_RED if avoiding_colour == 1 else TARGET_OFFSET_GREEN
                error = obs_x - target
                obstacle_steer_bias = KP_OBSTACLE * error
            # else: no contour this frame -> keep the last bias rather than
            # snapping back to 0 on a single dropped detection
    else:
        obstacle_steer_bias = 0

    # STEERING CALCULATION -------------------

    steering = 100 + navigate_wall(gyro, desired_heading, left_area=left_area, right_area=right_area, obstacle_bias=obstacle_steer_bias)  # blended gyro+camera+obstacle steering, offset for serial protocol



    # CHECKING FOR TURN MARKERS -------------------

    # Only look for a new turn-colour line if we're outside the "just turned" cooldown window.
    if not turning:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours()
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours) # if bottom_colour = 1 = blue if bottom_colour = 2 = orange
        # print("GOT THE AREAS")


        # A large enough patch of blue/orange counts as a line crossing.
        if bottom_area > 800:
            # print(time.time(), "--Detected turn color--")

            # First detection ever: lock in direction AND the colour we care about.
            # After this, the other colour is completely ignored for the rest of the run.
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"   # blue seen first -> locked to blue/left forever
                elif bottom_colour == 2:
                    direction = "CWR"   # orange seen first -> locked to orange/right forever

            # Only react to a detection if it matches the locked-in colour.
            # (direction == "CCL" <-> blue, direction == "CWR" <-> orange)
            turning_time = time.time()  # restart the cooldown window
            turning = True  # This is only used for debug purposes to indicate end of turn
            if direction == "CCL":
                blue_count += 1
                print(f"BLUE: {blue_count}")
            else:
                orange_count += 1
                print(f"ORANGE: {orange_count}")

            pending_turn = True
            cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # else: this is the "other" colour showing up after lock-in — ignored entirely.
    else:
        # Debug: mark end of turn windowq
        if time.time() - turning_time > 1.5: 
            turning = False


    if pending_turn: # if a turn is pending, execute it and reset the pending flag
        left_frame.update(cap)
        right_frame.update(cap)


        left_contours = left_frame.find_contours()
        right_contours = right_frame.find_contours()


        left_area, _ = left_frame.get_areas(left_contours)
        right_area, _ = right_frame.get_areas(right_contours)


        # checked to make sure that it is safe to turn (i.e. not too close to a wall)
        if direction == "CCL":  # left
            if left_area < SAFE_TURN_AREA: # black area on left is small enough to turn left
                turn(direction)
                pending_turn = False


        elif direction == "CWR":  # right
            if right_area < SAFE_TURN_AREA: # black area on right is small enough to turn right
                turn(direction)
                pending_turn = False


    # Middle ROI: continuously scan for red, green, and black. Find whichever
    # single contour is largest across the three colours and show that colour.


    # Once either colour has been crossed LINE_COUNT times, start the stop sequence.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True
   
    if stop:
        if time.time() - stop_time > 2                                             :
            speed = 0000
            ser.write(f"19020000\n".encode())  # send the fixed stop command
            ser.flush()
            break




    if (SHOW_VID):
        # Overlay debug info (steering angle, line counts, FPS) on the preview frame.
        cv2.putText(cap, f"Steer: {steering:.2f}", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(cap, f"O: {str(orange_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,127,255), 1)
        cv2.putText(cap, f"B: {str(blue_count)}", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,127,0), 1)


        frame_count += 1
        elapsed = time.time() - frame_time
        if elapsed > 1.0:
            fps = frame_count // elapsed  # rough FPS sampled once per second
            frame_count = 0


            frame_time = time.time()
        cv2.putText(cap, f"FPS: {fps:.2f}", (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)            


        cv2.imshow("Video Frame", cap)



    if abs(sent_steer - steering) >=3:
        ser.write(f"{steering:.0f}{speed+1000}{controller}\n".encode())  # send steering+speed to the microcontroller each loop
        sent_steer = steering
        ser.flush()
    time.sleep(0.01)
    # print(f"sent value: heading: {steering} speed: {speed}")
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"19010230\n".encode())
        ser.flush()
        break

ser.write(f"19050000\n".encode())
ser.close()
       

cv2.destroyAllWindows()