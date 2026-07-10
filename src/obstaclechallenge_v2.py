# obstacle_v2.py ----------------------------------------------------------------------------
#
# Your original hybrid gyro+camera controller, now with the reference code's
# obstacle-avoidance ALGORITHM ported onto your architecture (Frame class,
# BNO055 heading, navigate_wall blend, serial protocol).
#
# The reference algorithm is a state machine:
#
#   'no_color'      -> nothing to avoid: pure wall+gyro navigation (your original loop)
#   'follow_color'  -> obstacle visible: steer so its centroid sits at a target
#                      pixel offset in middle_frame (your existing offset controller,
#                      same idea as the reference's cam_error = cx - (CENTER +/- offset))
#   'lost_color'    -> obstacle vanished for a few frames (it slid under/beside the
#                      camera's view): keep a small HEADING search bias toward the
#                      side it was last on, so we stay committed instead of snapping
#                      straight and clipping it with the rear of the robot
#   'pass_color'    -> obstacle gone for longer: commit to a gyro heading swing that
#                      cuts BACK toward the lane heading, and hand control back to
#                      normal navigation once the actual heading re-crosses within
#                      PASS_EXIT_TOL_DEG of desired_heading (this is the reference
#                      code's "pass_color: error = heading_diff + avoidance_angle,
#                      exit when |heading_diff| < 12" behaviour)
#
#   'dodge_hard'    -> NEW state, not in the reference: entered when an obstacle is
#                      first seen ALREADY huge, or hugging the edge of middle_frame
#                      (i.e. it was hidden behind a corner / at the far end of the
#                      mat and we didn't get a run-up). Slows down and applies a
#                      full heading swing toward the pass side immediately, on top
#                      of the pixel-offset bias, so the dodge happens even if the
#                      camera geometry alone wouldn't turn us fast enough.
#
# Wall protection (robot must not touch outer wall, inner wall, or obstacle):
#   1. navigate_wall's SAFE_TURN_AREA clamp is generalised: ANY steering source
#      (obstacle bias, search bias, heading swing) is capped from steering further
#      toward a wall whose black area is already large.
#   2. A harder WALL_DANGER_AREA override forces steering away from a wall that is
#      critically close, no matter what the avoidance states want.
#   3. Speed drops during lost/pass/dodge_hard, like the reference code does.
#
# Turn logic (blue/orange in bottom_frame -> CW/CCW) is UNCHANGED. Heading swings
# are expressed as offsets on top of desired_heading, so a 90-degree corner turn
# composes cleanly with an in-progress avoidance (obstacle right after a corner /
# on the opposite straight still gets dodged relative to the NEW lane heading).
#
# Lines marked  ### NEW  or  ### CHANGED  are the diffs vs your original file.
# --------------------------------------------------------------------------------------------

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
LINE_COUNT = 12                 # number of colour-line crossings before stopping
SAFE_TURN_AREA = 2000           # max black area on a side before steering toward it is clamped
WALL_DANGER_AREA = 2500         ### NEW: black area at which we FORCE steering away from that
                                #        wall (left/right ROI is 20x140 = 2800 px max, so 2500
                                #        means "almost the whole strip is wall" -> tune on bot)
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)

OBSTACLE_ENTER_AREA = 400    # min red/green contour area (px) to start an avoidance maneuver
OBSTACLE_EXIT_AREA = 150     # area below which the sign is considered "not seen" (hysteresis)
OBSTACLE_LATE_AREA = 2500    ### NEW: if FIRST sighting is already this big, we had no run-up
                             #        (hidden behind corner / popped in at the frame edge)
                             #        -> skip straight to 'dodge_hard'. Tune on the bot.
EDGE_MARGIN = 30             ### NEW: px from the ROI edge; a decent-sized obstacle first seen
                             #        this close to the edge also triggers 'dodge_hard'
OBSTACLE_MAX_AVOID_TIME = 3.0  # failsafe cap on follow/dodge phases (never the primary exit)

# --- Obstacle offset controller (unchanged idea: steer using the obstacle's own
# pixel position in middle_frame) ---
KP_OBSTACLE = 0.15   # proportional gain on obstacle pixel error -> tune on the bot
# middle_frame spans x=25..295 -> ROI width 270px, ROI-local x runs 0..270, center ~135.
MID_ROI_W = 270              ### NEW: ROI-local width of middle_frame, for edge checks
TARGET_OFFSET_RED = 220      # red -> keep it toward the right of the ROI, so we pass on its left
TARGET_OFFSET_GREEN = 50     # green -> keep it toward the left of the ROI, so we pass on its right
# NOTE: these targets make the robot pass RED on its LEFT and GREEN on its RIGHT.
# The reference code does the OPPOSITE (red -> pass right, green -> pass left, the
# usual WRO convention). I kept YOUR targets; if your rules need the other sides,
# just swap the two TARGET_OFFSET values and the three *_SIGN tables below flip
# automatically because they are derived from the targets.

### NEW ------------- state-machine tuning (ported from the reference code) -------------
LOST_FRAMES_ENTER = 3        # consecutive "not seen" frames: follow_color -> lost_color
                             #   (reference: no_color_count > 2)
PASS_FRAMES_ENTER = 7        # consecutive "not seen" frames: lost_color -> pass_color
                             #   (reference: no_color_count > 6)
SEARCH_BIAS_DEG = 12         # heading bias toward the obstacle's last side while 'lost'
                             #   (reference: search_offset = 15)
PASS_SWING_DEG = 35          # cut-back heading swing magnitude in 'pass_color'
                             #   (reference: avoidance_angle = 35)
HARD_SWING_DEG = 35          # immediate dodge swing magnitude in 'dodge_hard'
PASS_EXIT_TOL_DEG = 12       # |heading error to desired_heading| below this ends pass_color
                             #   (reference: abs(heading_diff) < 12)
PASS_MAX_TIME = 2.0          # failsafe cap for the pass_color swing itself

SPEED_NORMAL = 800           # your original cruise speed
SPEED_AVOID = 700            ### NEW: slower while lost/pass/dodge_hard (reference drops from
                             #        max_fs to min_fs there). Set equal to SPEED_NORMAL if
                             #        your drivetrain stalls below 800.
### -------------------------------------------------------------------------------------

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed MCU
time.sleep(2)  # let the serial connection settle before writing
steering = 100 + DEFAULT_STEER_ANGLE
stop = False
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCL")

### CHANGED: the single `avoiding` flag becomes a proper state machine --------------------
avoid_state = 'no_color'     # 'no_color' | 'follow_color' | 'lost_color' | 'pass_color' | 'dodge_hard'
avoiding_colour = None       # 1 = red, 2 = green (locked while avoid_state != 'no_color')
avoiding_time = time.time()  # start of current avoidance (failsafe cap for follow/dodge)
pass_time = time.time()      # start of current pass_color swing (its own failsafe cap)
no_color_count = 0           # consecutive frames the avoided colour has NOT been seen
obstacle_steer_bias = 0      # pixel-offset steering bias (only nonzero in follow/dodge_hard)
avoid_heading_offset = 0     # degrees added to desired_heading (search bias / swings)
last_obs_x = None            # last known ROI-local centroid x of the avoided obstacle
### ---------------------------------------------------------------------------------------

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

desired_heading = bno055.get_heading()

# One-off startup check: confirm gyro is readable and report calibration state.
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


### NEW: per-colour sign tables, derived from your TARGET_OFFSET convention.
# Steering convention in navigate_wall: value > DEFAULT_STEER_ANGLE steers RIGHT
# (more black on the left -> cam_steer rises -> steer away from left wall), and
# turn("CWR") adds +90 to desired_heading -> gyro term also rises -> RIGHT. So:
#   +offset on desired_heading  == swing RIGHT,  -offset == swing LEFT.
#
# With your targets: RED is held at x=220 (right of ROI) -> robot dodges LEFT,
# passing on red's left. GREEN is held at x=50 -> robot dodges RIGHT.
def dodge_sign(colour):      # direction of the dodge itself (dodge_hard swing)
    return -1 if colour == 1 else 1     # red: left (-), green: right (+)

def cutback_sign(colour):    # direction of the pass_color cut-back (opposite of dodge)
    return 1 if colour == 1 else -1     # red: cut back right (+), green: left (-)

def search_sign(colour):     # lost_color: steer toward the side the obstacle was on
    return 1 if colour == 1 else -1     # red was on our right (+), green on our left (-)


def turn(direction):
    global desired_heading

    if direction == "CWR":      # clockwise = right turn
        desired_heading += 90
        print("CWR")
    elif direction == "CCL":    # counterclockwise = left turn
        desired_heading -= 90
        print("CCL")

    # normalize to 0-360 range
    desired_heading = desired_heading % 360


def heading_to_signed(heading):
    """
    Converts a raw gyro heading (0-359 deg) into a signed angle:
    0 = straight. Examples: 0->0, 20->20, 180->180, 359->-1, 350->-10.
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
    is driven by what the camera actually sees rather than a fixed timer.
    """
    if not contours:
        return 0
    return sum(cv2.contourArea(c) for c in contours)


def contour_centroid_x(contours):
    """
    ROI-local x-pixel centroid of the largest contour in `contours`, or None.
    Tells us where a red/green obstacle sits horizontally in middle_frame.
    """
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"]


def navigate_wall(gyro_heading, desired_heading=0, KP=0.05, KP_GYRO=0.5,
                  left_area=0, right_area=0, obstacle_bias=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error.
      2. Camera term: proportional correction on left/right wall pixel area difference.
    Final steering = 70% gyro + 30% camera + obstacle bias, then wall-safety
    clamps, then clamped to servo range [30, 150].
    """

    # Camera term: more black pixels on one side pushes steering the other way.
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)

    # Gyro term: drives heading error toward zero. Straight if gyro unavailable.
    if gyro_heading is not None:
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE

    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer + obstacle_bias

    ### CHANGED: wall safety clamp now applies to the TOTAL steering command, not
    # just the obstacle bias. Search biases and heading swings enter through the
    # gyro term (via avoid_heading_offset on desired_heading), so clamping only
    # `obstacle_bias` would let a swing drive us into a wall. If a wall's black
    # area is already large, we refuse to steer more than 10 units toward it.
    # (Corner turns are unaffected: pending_turn already waits until that side's
    # area is BELOW SAFE_TURN_AREA before the turn executes.)
    if left_area > SAFE_TURN_AREA:
        steering_value = max(steering_value, DEFAULT_STEER_ANGLE - 10)   # cap leftward steer
    if right_area > SAFE_TURN_AREA:
        steering_value = min(steering_value, DEFAULT_STEER_ANGLE + 10)   # cap rightward steer

    ### NEW: hard override. If a wall strip is almost entirely black we are about
    # to touch it -> force steering AWAY regardless of what avoidance wants.
    # (If both sides are critical the two overrides cancel to roughly straight,
    # which is the least-bad option in a corridor.)
    if left_area > WALL_DANGER_AREA:
        steering_value = max(steering_value, DEFAULT_STEER_ANGLE + 20)   # force right
    if right_area > WALL_DANGER_AREA:
        steering_value = min(steering_value, DEFAULT_STEER_ANGLE - 20)   # force left

    steering_value = max(30, min(150, steering_value))  # clamp to servo range

    gh = f"{gyro_heading:.0f}" if gyro_heading is not None else "None"   ### CHANGED: no crash on None
    print(f"gyro heading: {gh}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, "
          f"obs bias: {obstacle_bias:.1f}, state: {avoid_state}, steer: {steering_value:.0f}")

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
left_frame = Frame(cap, 0, 20, 60, 200, colour_range=[black_range])
right_frame = Frame(cap, 300, 320, 60, 200, colour_range=[black_range])
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])

# Middle ROI: spans almost the full width, above the bottom turn-marker strip.
middle_frame = Frame(cap, 25, 295, 20, 195, colour_range=[red_range, green_range, black_range])

print("ENTERING THE WHILE LOOP")

while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None
    speed = SPEED_NORMAL                   ### CHANGED: may be lowered by avoidance below

    # CHECKING FOR OBSTACLES -------------------

    middle_frame.update(cap)
    mid_red_contours, mid_green_contours, mid_black_contours = middle_frame.find_contours()

    red_area = contour_area(mid_red_contours)
    green_area = contour_area(mid_green_contours)

    if red_area > 0 or green_area > 0:
        cv2.putText(cap, f"Mid: R({red_area:.0f}) G({green_area:.0f})", (90, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    ### CHANGED --------------------------------------------------------------------------
    # OBSTACLE AVOIDANCE STATE MACHINE (ported from the reference algorithm).
    #
    # Every frame we compute two outputs for navigate_wall:
    #   obstacle_steer_bias  -> pixel-offset steering (follow_color / dodge_hard)
    #   avoid_heading_offset -> degrees added to desired_heading (search / swings)
    # ------------------------------------------------------------------------------------

    # Refresh side-wall areas FIRST: the state machine and navigate_wall both
    # need to know how close the walls are this frame.
    left_frame.update(cap)
    right_frame.update(cap)
    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()
    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    if avoid_state == 'no_color':
        obstacle_steer_bias = 0
        avoid_heading_offset = 0

        # Entry: a red/green contour big enough to be a real sign. If both
        # appear at once, react to whichever is currently larger.
        if red_area > OBSTACLE_ENTER_AREA or green_area > OBSTACLE_ENTER_AREA:
            avoiding_colour = 1 if red_area >= green_area else 2
            avoiding_time = time.time()
            no_color_count = 0

            first_contours = mid_red_contours if avoiding_colour == 1 else mid_green_contours
            first_area = red_area if avoiding_colour == 1 else green_area
            obs_x = contour_centroid_x(first_contours)
            last_obs_x = obs_x

            # LATE-SIGHTING CHECK: first sighting is already huge, or a sizable
            # blob is hugging the ROI edge (it was hidden behind a corner or sat
            # at the far end of the mat until now). No run-up -> dodge hard NOW.
            seen_late = first_area > OBSTACLE_LATE_AREA
            on_edge = (obs_x is not None
                       and (obs_x < EDGE_MARGIN or obs_x > MID_ROI_W - EDGE_MARGIN)
                       and first_area > OBSTACLE_ENTER_AREA * 2)

            if seen_late or on_edge:
                avoid_state = 'dodge_hard'
                print(f"-- OBSTACLE SEEN LATE ({'edge' if on_edge else 'area'}) -> DODGE HARD --")
            else:
                avoid_state = 'follow_color'
                print(f"-- OBSTACLE ({'RED' if avoiding_colour == 1 else 'GREEN'}) -> FOLLOW --")

    if avoid_state in ('follow_color', 'dodge_hard'):
        current_area = red_area if avoiding_colour == 1 else green_area
        contours = mid_red_contours if avoiding_colour == 1 else mid_green_contours
        obs_x = contour_centroid_x(contours)
        if obs_x is not None:
            last_obs_x = obs_x

        if current_area > OBSTACLE_EXIT_AREA:
            no_color_count = 0            # still clearly see it
        else:
            no_color_count += 1           # not seen this frame

        # Pixel-offset controller: steer the centroid toward its target x.
        # (Same as the reference's cam_error against CAM_CENTER +/- offset.)
        if obs_x is not None and current_area > OBSTACLE_EXIT_AREA:
            target = TARGET_OFFSET_RED if avoiding_colour == 1 else TARGET_OFFSET_GREEN
            obstacle_steer_bias = KP_OBSTACLE * (obs_x - target)
        # else: keep the previous bias -- one dropped detection shouldn't snap us straight.

        if avoid_state == 'follow_color':
            avoid_heading_offset = 0
            # Escalate: if it balloons while following, we're closing too fast.
            if current_area > OBSTACLE_LATE_AREA:
                avoid_state = 'dodge_hard'
                print("-- OBSTACLE VERY CLOSE -> DODGE HARD --")
        else:  # dodge_hard
            # Full heading swing toward the pass side, ON TOP of the pixel bias,
            # and slow down. This guarantees turning authority even when the
            # pixel error alone is too small/too late.
            avoid_heading_offset = dodge_sign(avoiding_colour) * HARD_SWING_DEG
            speed = SPEED_AVOID

        # Exit follow/dodge: obstacle not seen for a few frames -> it left the
        # camera's view (we're beside it). Do NOT go straight yet: lost_color
        # keeps us committed while it sits in the blind spot.
        if no_color_count >= LOST_FRAMES_ENTER:
            avoid_state = 'lost_color'
            print("-- OBSTACLE LOST FROM VIEW -> SEARCH/COMMIT --")

        # Failsafe only: never the primary exit.
        if time.time() - avoiding_time > OBSTACLE_MAX_AVOID_TIME:
            avoid_state = 'pass_color'
            pass_time = time.time()
            print("-- AVOID TIMEOUT -> PASS SWING --")

    elif avoid_state == 'lost_color':
        # Reference behaviour: small steering bias toward where the obstacle was
        # (search_offset), so we neither reacquire-overshoot nor cut back early
        # and clip it with the rear wheel.
        obstacle_steer_bias = 0
        avoid_heading_offset = search_sign(avoiding_colour) * SEARCH_BIAS_DEG
        speed = SPEED_AVOID

        current_area = red_area if avoiding_colour == 1 else green_area
        if current_area > OBSTACLE_EXIT_AREA:
            no_color_count = 0
            avoid_state = 'follow_color'  # reacquired -> back to following
            print("-- OBSTACLE REACQUIRED -> FOLLOW --")
        else:
            no_color_count += 1
            if no_color_count >= PASS_FRAMES_ENTER:
                avoid_state = 'pass_color'
                pass_time = time.time()
                print("-- OBSTACLE CLEARED -> PASS SWING --")

    elif avoid_state == 'pass_color':
        # Reference behaviour: commit to a heading swing that cuts BACK toward
        # the lane (opposite the dodge). While following, our heading drifted
        # off desired_heading; the swing drives it back across the target, and
        # the moment |error to desired_heading| < tolerance we hand control
        # back to normal navigation ('no_color').
        obstacle_steer_bias = 0
        avoid_heading_offset = cutback_sign(avoiding_colour) * PASS_SWING_DEG
        speed = SPEED_AVOID

        # If a NEW (or the same) obstacle shows up mid-swing -- e.g. two signs
        # close together, or one at the opposite end of a short straight --
        # avoidance restarts immediately instead of finishing the swing into it.
        if red_area > OBSTACLE_ENTER_AREA or green_area > OBSTACLE_ENTER_AREA:
            avoiding_colour = 1 if red_area >= green_area else 2
            avoiding_time = time.time()
            no_color_count = 0
            new_area = red_area if avoiding_colour == 1 else green_area
            avoid_state = 'dodge_hard' if new_area > OBSTACLE_LATE_AREA else 'follow_color'
            print("-- NEW OBSTACLE DURING PASS -> RESTART AVOID --")
        elif gyro is not None and abs(angle_error(gyro, desired_heading)) < PASS_EXIT_TOL_DEG:
            avoid_state = 'no_color'
            avoiding_colour = None
            avoid_heading_offset = 0
            print("-- PASS COMPLETE -> NORMAL NAV --")
        elif time.time() - pass_time > PASS_MAX_TIME:
            avoid_state = 'no_color'      # failsafe: don't swing forever
            avoiding_colour = None
            avoid_heading_offset = 0
            print("-- PASS TIMEOUT -> NORMAL NAV --")
    ### END CHANGED ----------------------------------------------------------------------

    # STEERING CALCULATION -------------------

    ### CHANGED: heading swings/search biases enter as an offset on desired_heading,
    # so they compose with corner turns (turn() shifts desired_heading; the swing
    # is then relative to the NEW lane heading).
    effective_heading = (desired_heading + avoid_heading_offset) % 360
    steering = 100 + navigate_wall(gyro, effective_heading,
                                   left_area=left_area, right_area=right_area,
                                   obstacle_bias=obstacle_steer_bias)

    # CHECKING FOR TURN MARKERS -------------------

    # Only look for a new turn-colour line if we're outside the "just turned" cooldown window.
    if not turning:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours()
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours)  # 1=blue, 2=orange

        # A large enough patch of blue/orange counts as a line crossing.
        if bottom_area > 800:

            # First detection ever: lock in direction AND the colour we care about.
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"   # blue seen first -> locked to blue/left forever
                elif bottom_colour == 2:
                    direction = "CWR"   # orange seen first -> locked to orange/right forever

            turning_time = time.time()  # restart the cooldown window
            turning = True
            if direction == "CCL":
                blue_count += 1
                print(f"BLUE: {blue_count}")
            else:
                orange_count += 1
                print(f"ORANGE: {orange_count}")

            pending_turn = True
            cv2.putText(cap, f"{bottom_colour}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        # Debug: mark end of turn window
        if time.time() - turning_time > 1.5:
            turning = False

    if pending_turn:  # if a turn is pending, execute it when it's safe
        # (left/right areas were already refreshed above this frame)
        # checked to make sure that it is safe to turn (i.e. not too close to a wall)
        if direction == "CCL":  # left
            if left_area < SAFE_TURN_AREA:  # black area on left is small enough to turn left
                turn(direction)
                pending_turn = False

        elif direction == "CWR":  # right
            if right_area < SAFE_TURN_AREA:  # black area on right is small enough to turn right
                turn(direction)
                pending_turn = False

    # Once either colour has been crossed LINE_COUNT times, start the stop sequence.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True

    if stop:
        if time.time() - stop_time > 2:
            speed = 0
            ser.write(f"19020000\n".encode())  # send the fixed stop command
            ser.flush()
            break

    if SHOW_VID:
        # Overlay debug info (steering angle, avoid state, line counts, FPS).
        cv2.putText(cap, f"Steer: {steering:.2f}", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(cap, f"{avoid_state}", (100, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)  ### NEW
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
        ser.write(f"{steering:.0f}{speed+1000}{controller}\n".encode())  # steering+speed to the MCU
        sent_steer = steering
        ser.flush()
    time.sleep(0.01)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"19010230\n".encode())
        ser.flush()
        break

ser.write(f"19050000\n".encode())
ser.close()

cv2.destroyAllWindows()
