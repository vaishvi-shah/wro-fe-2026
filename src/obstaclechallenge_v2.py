# 1 ----------------------------------------------------------------------------------------
#
# WORKING FINAL COPY + obstacle avoidance (ported from the anti-obstacle algorithm)
#
# Base behaviour (UNCHANGED): 70% gyro / 30% camera wall-follow, blue/orange line
# counting in the bottom frame -> 90 degree corner turns, stop after 12 lines.
#
# ADDED (tagged ### NEW / ### CHANGED so you can diff against your working copy):
#   * Red/green obstacle avoidance as a state machine ported from the reference:
#       - GREEN is passed on its LEFT  (robot dodges LEFT, keeps green on the RIGHT of view)
#       - RED   is passed on its RIGHT (robot dodges RIGHT, keeps red  on the LEFT  of view)
#     States: no_color -> follow_color -> (dodge_hard) -> lost_color -> pass_color -> no_color
#   * A dodge composes with a corner turn: swings are offsets ON TOP of desired_heading,
#     and both the turn detector and the obstacle detector run EVERY (non-reverse) frame,
#     so a turn is never missed because of a dodge and a dodge is never missed because of
#     a turn. A pending turn stays latched until its side is safe, so it can't be lost.
#   * Safety reverse (controller = 2): if the robot is about to scrape the black wall,
#     OR it has bumped an obstacle and is dragging it (blob stays at ~3/7 of the ROI),
#     it reverses to regain clearance, then re-commits to the dodge / resumes navigation.
#     This is what stops the "hit it, don't know, drag the blob along" failure.
# --------------------------------------------------------------------------------------------

'''
#1 - Hybrid weighted control (gyro + camera as two separate steering inputs)
Camera and gyro each compute their own steering value independently, blended 70/30.
Obstacle avoidance sits on top as a heading-offset + pixel-bias layer, and a reverse
escape overrides everything when a collision with the wall or an obstacle is imminent.
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
controller = 1                 # 1 = drive forward, 2 = REVERSE (used by the safety escape)
sent_steer = 0
last_controller = 1            ### NEW: force a serial send whenever the controller flips
last_speed = None              ### NEW: force a serial send whenever the speed changes
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent as 100 + this
LINE_COUNT = 12                # number of colour-line crossings before stopping
SAFE_TURN_AREA = 2000           # max black area on the side of a turn before we can safely turn
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)

SPEED_NORMAL = 800   # your fast cruise speed
SPEED_AVOID  = 800   ### NEW: speed while actively dodging. Kept fast per your setup; drop to
                     #        ~650-700 if the bot understeers into obstacles at full speed.
REVERSE_SPEED = 600  ### NEW: magnitude used while backing out of a collision (controller = 2)

# ---- Obstacle colour ranges (HSV). Red wraps the 0/180 hue boundary -> two sub-ranges. ----
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

### NEW ---- Obstacle-avoidance geometry & tuning ------------------------------------------
# middle_frame ROI (defined below): x = 25..295 (width 270), y = 20..195 (height 175).
MID_ROI_X0, MID_ROI_X1 = 25, 295
MID_ROI_Y0, MID_ROI_Y1 = 20, 195
MID_ROI_W = MID_ROI_X1 - MID_ROI_X0          # 270  -> ROI-local x runs 0..270, centre ~135
MID_ROI_H = MID_ROI_Y1 - MID_ROI_Y0          # 175
MID_ROI_AREA = MID_ROI_W * MID_ROI_H         # 47250 px = full ROI

# The obstacle can take up AT MOST 3/7 of the camera, and (per your setup) that only
# happens once the robot has actually TOUCHED it. So an area at/above 3/7 == contact.
CONTACT_AREA = int(MID_ROI_AREA * 3 / 7)     # ~20250 px -> "we've hit it / are dragging it"
CLOSE_AREA   = int(MID_ROI_AREA * 0.18)      # ~8500 px  -> obstacle very close: dodge hard
OBSTACLE_ENTER_AREA = 400     # min red/green area to START reacting (filters noise)
OBSTACLE_EXIT_AREA  = 150     # area below which the sign is considered "not seen" (hysteresis)
OBSTACLE_LATE_AREA  = CLOSE_AREA   # first sighting already this big -> no run-up -> dodge hard
EDGE_MARGIN = 30              # px from ROI edge; a chunky sign first seen this close to the
                             # edge (hidden behind a corner / far end of mat) -> dodge hard

KP_OBSTACLE = 0.15           # gain on obstacle pixel error -> tune on the bot
# ROI-local x targets (centre ~135). RED kept LEFT so we pass on its right; GREEN kept
# RIGHT so we pass on its left. (Swap these two if a rule ever flips.)
TARGET_OFFSET_RED   = 75     # red  -> hold toward left of ROI  -> robot dodges RIGHT
TARGET_OFFSET_GREEN = 195    # green -> hold toward right of ROI -> robot dodges LEFT

LOST_FRAMES_ENTER = 3        # "not seen" frames: follow_color -> lost_color  (ref: >2)
PASS_FRAMES_ENTER = 7        # "not seen" frames: lost_color  -> pass_color   (ref: >6)
SEARCH_BIAS_DEG   = 12       # heading bias toward the obstacle's side while 'lost' (ref: 15)
HARD_SWING_DEG    = 35       # immediate dodge swing in 'dodge_hard'
PASS_SWING_DEG    = 35       # cut-back swing toward the lane in 'pass_color' (ref: 35)
PASS_EXIT_TOL_DEG = 12       # |heading err to lane| below this ends pass_color (ref: <12)
PASS_MAX_TIME     = 2.0      # failsafe cap for the pass swing
OBSTACLE_MAX_AVOID_TIME = 3.0  # failsafe cap for follow/dodge (never the primary exit)

# Wall-collision reverse thresholds (side ROI max area = 20*140 = 2800 px).
WALL_DANGER_AREA  = 2500     # a side strip this black == about to scrape that wall
WALL_DANGER_FRAMES = 2       # require it for N frames so one noisy frame can't trigger reverse
CONTACT_FRAMES     = 2       # require CONTACT_AREA for N frames before calling it a bump
REVERSE_MIN_TIME = 0.25      # always back up at least this long once triggered
REVERSE_MAX_TIME = 1.20      # ... but never longer than this (failsafe)
### ----------------------------------------------------------------------------------------

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed MCU
time.sleep(2)  # let the serial connection settle before writing
steering = 100 + DEFAULT_STEER_ANGLE
stop = False
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCL")

### NEW ---- obstacle / reverse state ------------------------------------------------------
avoid_state = 'no_color'     # no_color | follow_color | dodge_hard | lost_color | pass_color
avoiding_colour = None       # 1 = red, 2 = green (locked while avoid_state != no_color)
avoiding_time = time.time()  # start of current avoidance (failsafe cap)
pass_time = time.time()      # start of current pass swing (its own failsafe cap)
no_color_count = 0           # consecutive frames the avoided colour has NOT been seen
obstacle_steer_bias = 0      # pixel-offset steering bias (follow_color / dodge_hard)
avoid_heading_offset = 0     # degrees added to desired_heading (search bias / swings)
last_obs_x = None            # last known ROI-local centroid x of the avoided obstacle

reversing = False            # true while the safety reverse escape is active
reverse_reason = None        # 'wall' or 'obstacle'
reverse_start = time.time()
reverse_resume_state = 'no_color'  # state to fall back into once we've backed off
reverse_pass_sign = 1        # +1 dodge right, -1 dodge left (for post-reverse re-commit)
wall_danger_count = 0        # consecutive frames a wall was critically close
contact_count = 0            # consecutive frames an obstacle blob was at CONTACT_AREA
### ----------------------------------------------------------------------------------------

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

desired_heading = bno055.get_heading()

# One-off startup check: confirm gyro is readable and report calibration state before the loop.
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


### NEW: per-colour sign helpers. In navigate_wall a HIGHER steering value == steer RIGHT
# (a right/CWR turn raises desired_heading, which raises gyro_steer). So a POSITIVE heading
# offset == swing right, NEGATIVE == swing left.
def dodge_sign(colour):      # direction we steer to get around it
    return 1 if colour == 1 else -1     # red -> right (+), green -> left (-)

def cutback_sign(colour):    # pass_color: cut back toward the lane (opposite the dodge)
    return -1 if colour == 1 else 1     # red -> back left (-), green -> back right (+)
# search bias (lost_color) keeps committing to the dodge, so it == dodge_sign.


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
    Converts a raw gyro heading (0-359) into a signed angle: 0 = straight,
    positive = left, negative = right. Examples: 0->0, 359->-1, 350->-10.
    """
    if heading <= 180:
        return heading
    return heading - 360


def angle_error(current, target):
    error = (current - target + 180) % 360 - 180
    return error


def contour_area(contours):
    """Total pixel area of a list of contours (0 if none)."""
    if not contours:
        return 0
    return sum(cv2.contourArea(c) for c in contours)


def contour_centroid_x(contours):
    """ROI-local x-pixel centroid of the largest contour, or None."""
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"]


def navigate_wall(gyro_heading, desired_heading=0, left_area=0, right_area=0, obstacle_bias=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro vs desired).
      2. Camera term: proportional correction on left/right wall pixel area difference.
    Then adds the obstacle pixel bias, applies a soft wall-safety clamp, and clamps
    to the servo range [30, 150]. (Wall areas are computed once in the main loop and
    passed in, so the reverse-escape logic and this function agree on what they see.)
    """
    # Camera term: more black on one side pushes steering away from that side.
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)

    # Gyro term: drives heading error toward zero. Straight if gyro unavailable.
    if gyro_heading is not None:
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE

    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer + obstacle_bias

    ### CHANGED: soft wall clamp now guards the TOTAL command (the obstacle bias and the
    # heading swings can both push toward a wall). If a wall is already close we refuse to
    # steer more than 10 units further into it. (Hard wall contact is handled by the reverse
    # escape in the main loop; this just keeps us off the wall in normal dodging.)
    if left_area > SAFE_TURN_AREA:
        steering_value = max(steering_value, DEFAULT_STEER_ANGLE - 10)   # cap leftward steer
    if right_area > SAFE_TURN_AREA:
        steering_value = min(steering_value, DEFAULT_STEER_ANGLE + 10)   # cap rightward steer

    steering_value = max(30, min(150, steering_value))  # clamp to servo range

    gh = f"{gyro_heading:.0f}" if gyro_heading is not None else "None"
    print(f"gyro:{gh} gyroS:{gyro_steer:.0f} camS:{cam_steer:.0f} "
          f"obs:{obstacle_bias:.1f} state:{avoid_state} steer:{steering_value:.0f}")
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

### NEW: middle ROI watches for red/green obstacles (and black, for reference). Spans almost
# the full width, above the bottom turn-marker strip, not covering the very top.
middle_frame = Frame(cap, MID_ROI_X0, MID_ROI_X1, MID_ROI_Y0, MID_ROI_Y1,
                     colour_range=[red_range, green_range, black_range])

print("ENTERING THE WHILE LOOP")

while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None
    speed = SPEED_NORMAL                   ### CHANGED: may be overridden by avoid / reverse

    # ---- Refresh side walls + middle obstacle view ONCE, up front, so every branch
    #      (navigation, obstacle state machine, reverse escape) sees the same data. ----
    left_frame.update(cap)
    right_frame.update(cap)
    left_area, _ = left_frame.get_areas(left_frame.find_contours())
    right_area, _ = right_frame.get_areas(right_frame.find_contours())

    middle_frame.update(cap)
    mid_red_contours, mid_green_contours, mid_black_contours = middle_frame.find_contours()
    red_area = contour_area(mid_red_contours)
    green_area = contour_area(mid_green_contours)
    biggest_ob_area = max(red_area, green_area)

    # ---- Collision bookkeeping (used to decide whether to reverse) -------------------- ### NEW
    if biggest_ob_area >= CONTACT_AREA:
        contact_count += 1        # blob at ~3/7 of the ROI == we've touched / are dragging it
    else:
        contact_count = 0
    if left_area >= WALL_DANGER_AREA or right_area >= WALL_DANGER_AREA:
        wall_danger_count += 1
    else:
        wall_danger_count = 0

    # ================================================================================
    # PRIORITY 1: SAFETY REVERSE (controller = 2). Overrides all navigation/turn logic.
    # Triggered by an imminent wall scrape, or by having bumped/dragged an obstacle.
    # ================================================================================
    if not reversing:
        if contact_count >= CONTACT_FRAMES:
            reversing = True
            reverse_reason = 'obstacle'
            reverse_start = time.time()
            # Re-commit to a hard dodge afterwards, on the side that clears the block.
            reverse_pass_sign = dodge_sign(1 if red_area >= green_area else 2)
            reverse_resume_state = 'dodge_hard'
            avoiding_colour = 1 if red_area >= green_area else 2
            print("-- OBSTACLE CONTACT / DRAGGING -> REVERSE --")
        elif wall_danger_count >= WALL_DANGER_FRAMES:
            reversing = True
            reverse_reason = 'wall'
            reverse_start = time.time()
            reverse_resume_state = avoid_state    # resume whatever we were doing
            # Back off steering AWAY from the close wall.
            reverse_pass_sign = 1 if left_area >= right_area else -1  # left close -> steer right
            print("-- WALL TOO CLOSE -> REVERSE --")

    if reversing:
        controller = 2                      # <-- reverse
        speed = REVERSE_SPEED
        # Steer while backing up: for a wall, point away from it; for an obstacle, lean
        # toward the pass side so we re-approach already offset.
        steer_angle = DEFAULT_STEER_ANGLE + reverse_pass_sign * 15
        steering = 100 + int(max(30, min(150, steer_angle)))

        elapsed_rev = time.time() - reverse_start
        cleared = (biggest_ob_area < CLOSE_AREA) if reverse_reason == 'obstacle' \
                  else (left_area < SAFE_TURN_AREA and right_area < SAFE_TURN_AREA)
        if elapsed_rev >= REVERSE_MIN_TIME and (cleared or elapsed_rev >= REVERSE_MAX_TIME):
            reversing = False
            controller = 1                  # <-- forward again
            contact_count = 0
            wall_danger_count = 0
            no_color_count = 0
            avoid_state = reverse_resume_state
            print(f"-- REVERSE DONE ({reverse_reason}) -> {avoid_state} --")

        # Send immediately during reverse (steering/speed/controller may all have changed),
        # skip navigation + turn detection this frame, keep the preview/quit responsive.
        if SHOW_VID:
            cv2.putText(cap, f"REVERSE ({reverse_reason})", (90, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("Video Frame", cap)
        ser.write(f"{steering:.0f}{speed+1000}{controller}\n".encode())
        ser.flush()
        sent_steer, last_controller, last_speed = steering, controller, speed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ser.write(f"19010230\n".encode()); ser.flush()
            break
        time.sleep(0.01)
        continue    # <-- do NOT run normal nav / turn logic while reversing

    # ================================================================================
    # PRIORITY 2: OBSTACLE AVOIDANCE STATE MACHINE (ported from the reference algorithm)
    # Produces obstacle_steer_bias (pixel) + avoid_heading_offset (degrees) each frame.
    # ================================================================================
    if red_area > 0 or green_area > 0:
        cv2.putText(cap, f"Mid R({red_area:.0f}) G({green_area:.0f})", (90, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if avoid_state == 'no_color':
        obstacle_steer_bias = 0
        avoid_heading_offset = 0
        # Entry: a red/green contour big enough to be a real sign. React to the larger.
        if red_area > OBSTACLE_ENTER_AREA or green_area > OBSTACLE_ENTER_AREA:
            avoiding_colour = 1 if red_area >= green_area else 2
            avoiding_time = time.time()
            no_color_count = 0
            first_contours = mid_red_contours if avoiding_colour == 1 else mid_green_contours
            first_area = red_area if avoiding_colour == 1 else green_area
            obs_x = contour_centroid_x(first_contours)
            last_obs_x = obs_x
            # Late sighting: already big, or a chunky blob hugging the ROI edge (it was
            # hidden behind a corner / sat at the far end of the mat). No run-up -> dodge now.
            seen_late = first_area > OBSTACLE_LATE_AREA
            on_edge = (obs_x is not None
                       and (obs_x < EDGE_MARGIN or obs_x > MID_ROI_W - EDGE_MARGIN)
                       and first_area > OBSTACLE_ENTER_AREA * 2)
            avoid_state = 'dodge_hard' if (seen_late or on_edge) else 'follow_color'
            print(f"-- {'RED' if avoiding_colour==1 else 'GREEN'} OBSTACLE -> {avoid_state} --")

    if avoid_state in ('follow_color', 'dodge_hard'):
        current_area = red_area if avoiding_colour == 1 else green_area
        contours = mid_red_contours if avoiding_colour == 1 else mid_green_contours
        obs_x = contour_centroid_x(contours)
        if obs_x is not None:
            last_obs_x = obs_x

        if current_area > OBSTACLE_EXIT_AREA:
            no_color_count = 0
        else:
            no_color_count += 1

        # Pixel-offset controller: steer the centroid toward its colour's target x.
        if obs_x is not None and current_area > OBSTACLE_EXIT_AREA:
            target = TARGET_OFFSET_RED if avoiding_colour == 1 else TARGET_OFFSET_GREEN
            obstacle_steer_bias = KP_OBSTACLE * (obs_x - target)
        # else keep last bias: one dropped frame shouldn't snap us straight into the block.

        if avoid_state == 'follow_color':
            avoid_heading_offset = 0
            if current_area > CLOSE_AREA:            # closing fast -> commit to a hard dodge
                avoid_state = 'dodge_hard'
                print("-- OBSTACLE CLOSE -> DODGE HARD --")
        else:  # dodge_hard: full heading swing toward the pass side + pixel bias, slower
            avoid_heading_offset = dodge_sign(avoiding_colour) * HARD_SWING_DEG
            speed = SPEED_AVOID

        # Exit follow/dodge only when it leaves view (we're beside it). Stay committed via
        # lost_color rather than snapping straight and clipping it with the rear wheel.
        if no_color_count >= LOST_FRAMES_ENTER:
            avoid_state = 'lost_color'
            print("-- OBSTACLE OUT OF VIEW -> COMMIT/SEARCH --")
        if time.time() - avoiding_time > OBSTACLE_MAX_AVOID_TIME:   # failsafe only
            avoid_state = 'pass_color'; pass_time = time.time()
            print("-- AVOID TIMEOUT -> PASS SWING --")

    elif avoid_state == 'lost_color':
        obstacle_steer_bias = 0
        avoid_heading_offset = dodge_sign(avoiding_colour) * SEARCH_BIAS_DEG  # keep committing
        speed = SPEED_AVOID
        current_area = red_area if avoiding_colour == 1 else green_area
        if current_area > OBSTACLE_EXIT_AREA:
            no_color_count = 0
            avoid_state = 'follow_color'          # reacquired
            print("-- OBSTACLE REACQUIRED -> FOLLOW --")
        else:
            no_color_count += 1
            if no_color_count >= PASS_FRAMES_ENTER:
                avoid_state = 'pass_color'; pass_time = time.time()
                print("-- OBSTACLE CLEARED -> PASS SWING --")

    elif avoid_state == 'pass_color':
        # Cut back toward the lane heading; exit the moment we've re-aligned.
        obstacle_steer_bias = 0
        avoid_heading_offset = cutback_sign(avoiding_colour) * PASS_SWING_DEG
        speed = SPEED_AVOID
        if red_area > OBSTACLE_ENTER_AREA or green_area > OBSTACLE_ENTER_AREA:
            # A second block appeared mid-swing (two ends of the mat / back-to-back signs):
            # restart avoidance immediately instead of swinging into it.
            avoiding_colour = 1 if red_area >= green_area else 2
            avoiding_time = time.time(); no_color_count = 0
            new_area = red_area if avoiding_colour == 1 else green_area
            avoid_state = 'dodge_hard' if new_area > OBSTACLE_LATE_AREA else 'follow_color'
            print("-- NEW OBSTACLE DURING PASS -> RESTART AVOID --")
        elif gyro is not None and abs(angle_error(gyro, desired_heading)) < PASS_EXIT_TOL_DEG:
            avoid_state = 'no_color'; avoiding_colour = None; avoid_heading_offset = 0
            print("-- PASS COMPLETE -> NORMAL NAV --")
        elif time.time() - pass_time > PASS_MAX_TIME:
            avoid_state = 'no_color'; avoiding_colour = None; avoid_heading_offset = 0
            print("-- PASS TIMEOUT -> NORMAL NAV --")

    # ---- STEERING: swings are offsets on top of desired_heading, so they COMPOSE with
    #      the 90-degree corner turns applied by turn(). ---------------------------------- ### CHANGED
    effective_heading = (desired_heading + avoid_heading_offset) % 360
    steering = 100 + navigate_wall(gyro, effective_heading,
                                   left_area=left_area, right_area=right_area,
                                   obstacle_bias=obstacle_steer_bias)

    # ================================================================================
    # PRIORITY 3: TURN DETECTION + EXECUTION (runs EVERY frame -> never skipped by a dodge)
    # ================================================================================
    if not turning:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours()
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours)  # 1=blue 2=orange

        if bottom_area > 800:   # a large enough patch counts as a line crossing
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
        if time.time() - turning_time > 1.5:
            turning = False

    if pending_turn:
        # A pending turn stays LATCHED until its side is clear, so a dodge that momentarily
        # bloats the wall area only DELAYS the turn -> the turn is never missed. (Areas were
        # already refreshed at the top of the loop.)
        if direction == "CCL" and left_area < SAFE_TURN_AREA:
            turn(direction); pending_turn = False
        elif direction == "CWR" and right_area < SAFE_TURN_AREA:
            turn(direction); pending_turn = False

    # ---- Stop after LINE_COUNT crossings ----
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True

    if stop:
        if time.time() - stop_time > 4:
            speed = 0
            ser.write(f"19020000\n".encode())  # send the fixed stop command
            ser.flush()
            break

    if SHOW_VID:
        cv2.putText(cap, f"Steer: {steering:.2f}", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(cap, f"{avoid_state}", (100, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)  ### NEW
        cv2.putText(cap, f"O: {str(orange_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 127, 255), 1)
        cv2.putText(cap, f"B: {str(blue_count)}", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 127, 0), 1)

        frame_count += 1
        elapsed = time.time() - frame_time
        if elapsed > 1.0:
            fps = frame_count // elapsed
            frame_count = 0
            frame_time = time.time()
        cv2.putText(cap, f"FPS: {fps:.2f}", (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Video Frame", cap)

    # ---- Send: on a steering change OR a speed/controller change (forces sends on
    #      avoid-speed drops and on returning to forward from reverse). --------------- ### CHANGED
    if abs(sent_steer - steering) >= 3 or speed != last_speed or controller != last_controller:
        ser.write(f"{steering:.0f}{speed+1000}{controller}\n".encode())
        sent_steer = steering
        last_speed = speed
        last_controller = controller
        ser.flush()
    time.sleep(0.01)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"19010230\n".encode())
        ser.flush()
        break

ser.write(f"19050000\n".encode())
ser.close()

cv2.destroyAllWindows()
