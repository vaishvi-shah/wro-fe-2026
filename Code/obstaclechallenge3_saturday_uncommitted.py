# 1 ----------------------------------------------------------------------------------------
"""
make the parking square size and location the sam I REMOVED THE THE 3 DEGREE THING -- CHECK CHECK CHECK
increased all speed by 7
"""


# imports!
import os
import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055
import board
# import adafruit_vl53l0x
from enum import Enum
import lib.motor_control as motor_control


class State(Enum):
    """Drive-mode only. TURNING is a first-class member and takes priority
    over WALL_FOLLOW/AVOIDING_OBSTACLE: as soon as a colour-line trips
    pending_turn, the main loop dispatches into run_turning() every frame
    instead of either of those, until the turn actually executes. REVERSING
    is still checked ahead of it, so an in-progress proximity-safety backup
    isn't cut short mid-manoeuvre -- see the dispatch order in the main
    loop."""
    OUT_PARKING = "OUT_PARKING"  # one-time manoeuvre at startup, before WALL_FOLLOW ever runs -- see out_parking_start
    WALL_FOLLOW = "WALL_FOLLOW"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    TURNING = "TURNING"  # executing a pending colour-line turn -- see pending_turn and run_turning()
    POST_OBS_VETO_TURN = "POST_OBS_VETO_TURN"  # brief gyro-only hold right after turn() fires from
                                                # the matching-obstacle-veto path in run_turning --
                                                # see run_post_obs_veto_turn()
    REVERSING = "REVERSING"
    IN_PARKING = "IN_PARKING"  # entered once LINE_COUNT is reached, instead of stopping -- NOT
                                # currently used (see FINISHING); logic kept, commented out, below  
    FINISHING = "FINISHING"  # entered once LINE_COUNT is reached instead of IN_PARKING: turn(),
                              # keep driving straight for FINISHING_DURATION, then stop()


CALIBRATION_FILE = "lib/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present

# ToF (VL53L0X) shares the same I2C bus as the IMU (IMU=0x28, ToF=0x29, see tof_tester.py) --
# activated right here alongside it, not lazily later, since board.I2C() must be grabbed
# once the bus is already up. Wrapped so a missing/failed ToF doesn't take the whole
# script down -- tof stays None and get_tof_distance() below just reports unavailab
# le.
tof = None
# try:
#     tof_i2c = board.I2C()
#     tof = adafruit_vl53l0x.VL53L0X(tof_i2c)
#     print(f"INFO: ToF (VL53L0X) Initialized. Distance: {tof.range} mm")
# except Exception as tof_e:
#     print(f"WARNING: ToF (VL53L0X) initialisation failed: {tof_e}")
#     tof = None


def get_tof_distance():
    """Returns the current ToF range in mm, or None if unavailable."""
    # print("ENTER get_tof_distance()")
    if tof:
        try:
            return tof.range
        except Exception:
            return None
    return None


controller = "FWD"
sent_steer = 0
sent_speed = None    # last speed actually sent -- a phase/state transition that changes speed or
sent_controller = None  # conqtroller (e.g. FWD->BWD) without also moving steering by >=3 must still
                        # resend, or the robot keeps executing whatever was last physically sent
SHOW_VID = os.environ.get("SHOW_VID", "1") != "0"  # togglsse live OpenCV preview window --
                                                    # defaults on when run directly; run.py
                                                    # sets SHOW_VID=0 to disable it via subprocess
                                                    
# SHOW_VID = False
DEFAULT_STEER_ANGLE = 82        # neutral/straight steering angle
STEER_MARGIN = 45
LINE_COUNT = 1              # number of colour-line crossings before stopping
MATCHING_OBSTACLE_CLEAR_DELAY = 0.2  # secqonds to keep gyro-crawling past a matching-colour obstacle
                                      # clearing the frame before actually firing turn()
MATCHING_OBSTACLE_VETO_SPEED = 75  # slower gyro-crawl speed while blocked by (or just clearing)
                                    # a matching-colour obstacle -- tune on t/.kklllllo90rack
POST_OBS_VETO_TURN_DURATION = 0.3  # seconds POST_OBS_VETO_TURN gyro-steers on the new heading

yellow_behaviour = None

KP = 0.012
#    # before handing off to WALL_FOLLOWKP = 0.008       # camera proportional gain (wall pixel area difference)
KD = 0.01    # camera derivative gain (damps oscillation from the wall pixel area difference)
KP_GYRO = 0.8    # gyro proportional gain (heading error in degrees)
KD_GYRO = 0.01  # gyro derivative gain (damps oscillation/overshoot from how fast the
                # heading error is changing), tune on track
KP_OBSTACLE = 0.4   # obstacle-avoidance proportional gain (target pixel error -> steering degrees)
KP_OBSTACLE_RED = 0.4  # RED-only override -- RED wasn't turning hard enough at the shared gain, tune on track
OBSTACLE_MIN_CONTOUR_AREA = 200  # min RED/GREEN contour size counted as a real obstacle
                                  # (was a bare 400 literal) -- lowered to pick up smaller/
                                  # farther-away blobs sooner, tune on track
OBSTACLE_REACHED_PX = 10  # |cam_error| below this counts as "reached" the other-colour obstacle's pass point
OBSTACLE_REACHED_PY = 10  # |cam_error_y| below this required too -- x can align long before the robot is actually alongside the point
# Two-sided wall tripwire: a fixed point near each bottom corner of the frame, checked every
# frame regardless of turn direction/state (WALL_FOLLOW, AVOIDING_OBSTACLE, TURNING,
# POST_OBS_VETO_TURN). If a point lands on black, steering is hard-overridden to deflect
# away from that side, overriding whatever the normal steering logic computed. Tune the
# point positions and deflection angles on track.
WALL_TRIPWIRE_LEFT = (15, 205)    # near bottom-left edge
WALL_TRIPWIRE_RIGHT = (305, 205)  # near bottom-right edge, mirror of the left point
WALL_TRIPWIRE_HARD_LEFT = 45   # hard-left steering override when the RIGHT point trips
WALL_TRIPWIRE_HARD_RIGHT = 135  # hard-right steering override when the LEFT point trips
PARK_SQUARE_SIZE = 10  # small debug squaQre drawn during IN_PARKING, tune on track
PARK_SQUARE_CENTER_Y = 75  # tune on track
STOP_AFTER_PARK_CLEARED = True  # TEMPORARY: hold once park_cleared instead of continuing into
                                 # the force turn -- flip to False to resume the rest of IN_PARKING

state = State.OUT_PARKING # authoritative: every branch below dispatches on this. Starts here so the
                            # out-parking manoeuvre runs before anything else; nothing ever sets state
                            # back to OUT_PARKING once it leaves, so it's guaranteed one-time.
stopping = False    # true once the stop sequence has started -- independent of `state`, which
                    # gets reassigned every frame by the obstacle-detection branch and would
                    # otherwise reset stop_time on every iteration
reversing_time = None  # timestamp REVERSING was entered
out_parking_start = None  # timestamp the OUT_PARKING manoeuvre began; None until the first OUT_PARKING frame
out_parking_steer = None  # locked-in max-steer value (opposite the closer wall) for the OUT_PARKING manoeuvre
out_parking_reverse_start = None  # timestamp the 0.5s reverse-away-from-the-matching-obstacle began --
                                   # None until a matching-colour obstacle (steer=40+GREEN or
                                   # steer=140+RED, i.e. the colour sitting on the side we're about to
                                   # swing into) is first seen after the 0.8s turn
out_parking_reversed = False  # true once that 0.5s reverse has completed -- switches to gyro-only
                               # forward driving, watching for the same obstacle to clear
out_parking_turn_called = False  # guards the one-shot turn() call, fired once on entering the
                                  # post-reverse gyro-steer phase, that sets its target heading to
                                  # desired_heading +90 (out_parking_steer==140) or -90 (==40)
out_parking_obstacle_seen_colour = None  # RED/GREEN colour that triggered the reverse, captured once
                                          # so the "wait until clear" check watches that same colour
out_parking_obs_clear_start = None  # timestamp the matching obstacle first read as gone from view
                                     # during the post-reverse gyro-steer phase -- same
                                     # MATCHING_OBSTACLE_CLEAR_DELAY gyro-crawl-then-commit pattern
                                     # run_turning uses for matching obstacles during a colour-line turn
out_parking_continue_start = None  # timestamp the further 0.2s straight-continue phase began, once
                                    # the matching obstacle's clear-delay above has elapsed
out_parking_original_heading = None  # desired_heading captured right before the escape turn() fires,
                                      # so it can be restored once the obstacle is cleared
out_parking_return_turn_called = False  # true once desired_heading has been reverted back to
                                         # out_parking_original_heading (the one-shot "turn back" step)
out_parking_return_turn_start = None  # timestamp the post-revert gyro-only settle hold began

park_square_filled = False
park_square_filled_time = None  # timestamp (195, 58) was first read as black -- the park_cleared
                                 # watch point below isn't checked/drawn until 1s after this
park_turn_called = False
park_turned = False
park_cleared = False
park_cleared_time = None
park_final_reverse_called = False  # guards the one-shot driveRotations() reverse-away call
                                    # fired once park_cleared is reached

FINISHING_DURATION = 2.0  # seconds FINISHING keeps driving straight (post-turn()) before stop()
finishing_turn_called = False  # guards the one-shot turn(direction) call at the start of FINISHING
finishing_start = None  # timestamp the post-turn straight drive began -- None until turn() fires
finishing_stopped = False  # guards the one-shot stop() call once FINISHING_DURATION elapses

obs_on_screen = False
obstacle_color = "undefined"

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = DEFAULT_STEER_ANGLE
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)
last_turn_time = -5  # timestamp of the last executed turn; gates re-detecting the same patch of colour
matching_obstacle_seen = False  # true once run_turning has seen the matching-colour (veto)
                                 # obstacle during the current pending_turn window -- once it then
                                 # clears (obs_on_screen goes False), the robot keeps gyro-crawling
                                 # for MATCHING_OBSTACLE_CLEAR_DELAY (see obs_cleared_time) before
                                 # turn() fires and hands back to WALL_FOLLOW. Reset to False
                                 # wherever pending_turn is newly set to True (a fresh turn window).
obs_cleared_time = None  # timestamp the matching-colour obstacle first went out of view during the
                          # current pending_turn window -- None until then (see run_turning).
post_obs_veto_turn_start = None  # timestamp POST_OBS_VETO_TURN began -- see run_post_obs_veto_turn()
cam_error = None  # last known obstacle-avoidance horizontal pixel error; None until the avoidance block first sets it
cam_error_y = None  # last known obstacle-avoidance vertical pixel error (robot vs. target_y); same lifecycle as cam_error

prev_wall_error = None  # last frame's (left_area - right_area), for the KD term in navigate_wall
prev_wall_error_time = None  # timestamp prev_wall_error was captured

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCL")


frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()


obs_on_screen = False  # true whenever red/green obstacle area is above threshold, regardless of
                        # `state` -- purely "is an obstacle visible right now"
obs_colour = None  # "RED"/"GREEN"/None, saved alongside obs_on_screen each frame -- whichever
                    # colour has the larger area when obs_on_screen is True, else None


# defining colour ranges (HSV) used to mask each region of interest
blue_range = [
    [np.array([90, 50, 75]), np.array([130, 255, 255])]
]

orange_range = [     
    [np.array([6, 40, 70]), np.array([25, 255, 255])]
    # [np.array([0, 50, 75]), np.array([0, 255, 255])]
]

yellow_range = [
    # V floor raised from 70 to 100 -- below 90 it overlapped black_range's V<=90
    # ceiling, so a dim/shadowed yellow pixel was matching both masks and getting
    # contoured as black too. 100 keeps it above that ceiling, same gap white_range
    # already keeps.
    [np.array([25, 40, 100]), np.array([35, 255, 255])]
]

# S widened to the full 0-255 range -- any hue/tint counts as "black" as
# long as it's dark enough (V), so shadows and colour-cast dark surfaces
# (not just true desaturated gray/black) are included too.
# V widened from 60 to 90 to catch dimmer/shadowed black -- stays below
# white_range's V>=100 floor so the two don't overlap.
black_range = [
    [np.array([0, 0, 0]), np.array([180, 255, 90])]
]

red1_range = [
    [np.array([0, 50, 75]), np.array([8, 255, 255])]
]

red2_range = [
    [np.array([170, 40, 75]), np.array([180, 255, 255])]
]
red_obstacle_range = red1_range + red2_range   # one colour group, two HSV ranges (hue wraps at 0/180)

green_obstacle_range = [
    [np.array([45, 70, 70]), np.array([80, 255, 255])]
]

magenta_range = [  # parking-bay marker colour, same range as parking.py -- widened
                    # H/S/V floors and ceiling to catch more lighter/darker/off-hue pink
    [np.array([140, 25, 50]), np.array([179, 255, 255])]
]

# TEMPORARY DEBUG: hover-coordinate readout on the preview window -- remove once done tuning ROIs/points.
mouse_x, mouse_y = -1, -14

def _on_mouse_move(event, x, y, flags, param):
    # print(f"ENTER _on_mouse_move(event={event}, x={x}, y={y}, flags={flags}, param={param})")
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y

if SHOW_VID:
    cv2.startWindowThread()  # needed so cv2.imshow updates without blocking on this thread
    cv2.namedWindow("Video Frame")
    # cv2.setMouseCallback("Video Frame", _on_mouse_move)

# initializing the camera
print("-- INITIALIZING CAMERA --")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()
cap = picam2.capture_array("main")  # grab one frame to size the ROI frames below

def effective_colour(colour):
    """Maps a detected obstacle colour to how it should be reacted to.
    YELLOW is steered/avoided/veto'd exactly like RED or GREEN (perQ
    yellow_behaviour) -- but callers that just want to know/display what
    was actually seen should keep using the raw colour, not this."""
    return yellow_behaviour if colour == "YELLOW" else colour

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
    # print(f"ENTER turn(direction={direction})")
    print(" o o o o o o o o  o o oo o o o o o o o o oo o o o")
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
    # print(f"ENTER heading_to_signed(heading={heading})")
    if heading <= 180:
        return heading
    return heading - 360


def angle_error(current, target):
    # print(f"ENTER angle_error(current={current}, target={target})")
    error = (current - target + 180) % 360 - 180
    print(f"*** IMU - Tgt: {target} Cur: {current} Error: {error}")
    return error

gyro_prev_error = None    # last frame's heading error, for the derivative term
gyro_prev_time = None     # timestamp that error was captured
gyro_prev_target = None   # target_heading the above are tracking -- a new turn (different
                           # target) resets them, so a stale slope from the previous
                           # turn/phase doesn't spike the derivative term on frame 1
gyro_last_derivative = 0.0  # last computed derivative, kept only for the debug overlay

def gyro_only_steer(gyro_heading, target_heading):
    """Gyro-only PD steering, no camera term -- used during IN_PARKING where the camera
    wall-following isn't reliable. The derivative term damps oscillation/overshoot from
    how fast the heading error is changing, and resets whenever target_heading changes
    (i.e. a new turn) so it doesn't spike off a stale error from the previous target."""
    # print(f"ENTER gyro_only_steer(gyro_heading={gyro_heading}, target_heading={target_heading})")
    global gyro_prev_error, gyro_prev_time, gyro_prev_target, gyro_last_derivative
    if gyro_heading is None:
        return DEFAULT_STEER_ANGLE

    if target_heading != gyro_prev_target:
        gyro_prev_error = None
        gyro_prev_time = None
        gyro_prev_target = target_heading

    error = angle_error(gyro_heading, target_heading)

    now = time.time()
    derivative = 0.0
    if gyro_prev_error is not None and gyro_prev_time is not None:
        dt = now - gyro_prev_time
        if dt > 0:
            derivative = (error - gyro_prev_error) / dt
    gyro_prev_error = error
    gyro_prev_time = now
    gyro_last_derivative = derivative

    gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * error - KD_GYRO * derivative
    print(f"gyro_steer: {gyro_steer} gyro error: {error} derivative {derivative}")

    gyro_steer = int(max(DEFAULT_STEER_ANGLE - STEER_MARGIN, min(DEFAULT_STEER_ANGLE + STEER_MARGIN, gyro_steer)))

    return gyro_steer

def check_wall_tripwire(black_mask, point):
    """Returns True if `point` (an (x, y) tuple) lands on black in this frame's mask."""
    x, y = point
    h, w = black_mask.shape[:2]
    return 0 <= y < h and 0 <= x < w and bool(black_mask[y, x])

def navigate_wall(gyro_heading, desired_heading=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro_heading vs desired_heading).
      2. Camera term: proportional correction on left/right wall pixel area difference (original logic).
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [45, 135] (90 +/- 45).
    """
    # print(f"ENTER navigate_wall(gyro_heading={gyro_heading}, desired_heading={desired_heading})")
    global prev_wall_error, prev_wall_error_time

    # Refresh the side frames with the latest camera capture and re-run the
    # colour mask + contour detection so we know how much "wall" each side sees.
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours(SHOW_VID)
    right_contours = right_frame.find_contours(SHOW_VID)


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    print(f"left_area={left_area:.0f} right_area={right_area:.0f}")  # TEMP debug -- black pixel counts only

    # Camera term: proportional on the current wall-area gap, plus a derivative
    # term on how fast that gap is changing -- damps oscillation/overshoot from
    # the P term alone instead of waiting for the gap itself to grow.
    # wall_error = left_area - right_area
    # now = time.time()
    # if prev_wall_error is not None and prev_wall_error_time is not None:
    #     dt = now - prev_wall_error_time
    #     wall_error_deriv = (wall_error - prev_wall_error) / dt if dt > 0 else 0.0
    # else:
    #     wall_error_deriv = 0.0  # no prior sample yet -- first call contributes P only
    # prev_wall_error = wall_error
    # prev_wall_error_time = now

    # cam_steer = DEFAULT_STEER_ANGLE + KP * kp_scale * wall_error + KD * kp_scale * wall_error_deriv
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)


    # Gyro term: drives heading error toward zero. Falls back to straight if gyro unavailable.
    if gyro_heading is not None:
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE

    # Weighted blend of the two independent steering estimates.
    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer
    steering_value = int(max(DEFAULT_STEER_ANGLE - STEER_MARGIN, min(DEFAULT_STEER_ANGLE + STEER_MARGIN, steering_value))) # clamp to servo range

    gyro_heading_str = f"{gyro_heading:.0f}" if gyro_heading is not None else "None"
    print(f"WALL Bl   NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNEND heading: {gyro_heading_str}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, steer: {steering_value:.0f}")
    # cv2.putText(cap, f"gyro={DEFAULT_STEER_ANGLE}-{KP_GYRO}*{kp_scale}*{heading_error:.1f}={gyro_steer:.0f}",
    #             (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    # cv2.putText(cap, f"cam={DEFAULT_STEER_ANGLE}+{KP}*{kp_scale}*{wall_error:.0f}+{KD}*{kp_scale}*{wall_error_deriv:.0f}={cam_steer:.0f}",
    #             (10, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    # cv2.putText(cap, f"steer={GYRO_WEIGHT}*gyro+{CAM_WEIGHT}*cam={steering_value:.0f}",
    #             (10, 166), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)


    return int(steering_value)


# execution of main program


# initializing frames: each Frame watches a fixed region of interest (ROI) for a colour mask.
# left/right strips watch for the black wall; bottom strip watches for blue/orange turn markers.
# left_frame = Frame(cap, 0, 60, 60, 200, colour_range=[black_range])
# right_frame = Frame(cap, 260, 320, 60, 200, colour_range=[black_range])
# bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range, black_range])
# middle_frame = Frame(cap, 60, 260, 60, 220, colour_range=[red_obstacle_range, green_obstacle_range])
left_frame = Frame(cap, 0, 80, 60, 200, colour_range=[black_range])
right_frame = Frame(cap, 240, 320, 60, 200, colour_range=[black_range])
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])
middle_frame = Frame(cap, 0, 320, 40, 220, colour_range=[red_obstacle_range, green_obstacle_range, yellow_range])
# Narrowed to sit strictly between left_frame/right_frame (x:0-60/x:260-320) so the three
# ROIs don't overlap -- was full frame width so an obstacle drifting sideways during the
# avoidance turn wouldn't get clipped/lost near the edges; narrowing this back reintroduces
# that risk, tune/watch for it on track.

# Whole-screen ROI, only used once IN_PARKING starts -- replaces every other (specialised,
# strip-watching) Frame above for the rest of the run: watches the entire frame at once for
# red, green, black, and magenta, since during parking nothing is confined to a known strip.
parking_frame = Frame(cap, 0, 320, 0, 240, colour_range=[red_obstacle_range, green_obstacle_range,
                                                           black_range, magenta_range])

def stop():
    print("BATMOBILE STOPPING")
    time.sleep(0.01)
    motor_control.stopMotor()
    ser.write(f"90,0,STOP\n".encode())
    ser.flush()

def run_wall_follow(gyro, cap):
    """WALL_FOLLOW's own logic: plain wall-following steering, turn-line
    detection + execution, and watching for a new obstacle -- if one shows up,
    hands off to AVOIDING_OBSTACLE starting next frame (this frame still
    drives plain wall-follow steering, not obstacle-avoidance steering; that
    starts once AVOIDING_OBSTACLE's own branch runs next frame)."""
    global steering, speed, state, obs_on_screen, direction, \
           blue_count, orange_count, pending_turn, last_turn_time, \
           matching_obstacle_seen, obs_cleared_time

    # print(f"ENTER run_wall_follow(gyro={gyro})")

    steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
    speed = 75

    # Update the middle frame and check for obstacles
    middle_frame.update(cap)
    red_contours, green_contours, yellow_contours = middle_frame.find_contours(SHOW_VID)
    red_area, _ = middle_frame.get_areas(red_contours)
    green_area, _ = middle_frame.get_areas(green_contours)
    yellow_area, _ = middle_frame.get_areas(yellow_contours)
    # Fold yellow's area into whichever colour it reacts as, so every area-threshold
    # check below (obs_on_screen, the AVOIDING_OBSTACLE handoff) treats a yellow
    # obstacle exactly like a RED/GREEN one without needing its own comparisons.
    if yellow_behaviour == "RED":
        red_area += yellow_area
    elif yellow_behaviour == "GREEN":
        green_area += yellow_area
    # else (e.g. "NONE"): yellow_area contributes to neither -- yellow obstacles
    # are detected but ignored entirely for obstacle-reaction purposes.

    obs_on_screen = red_area > 100 or green_area > 100

    if not pending_turn and time.time() - last_turn_time > 1.5:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours(SHOW_VID)
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours) # if bottom_colour = 1 = blue if bottom_colour = 2 = orange

        # Black flooding the bottom frame means we're too close to a wall/obstacle in
        # front -- stop immediat    ely rather than keep driving into it.
        # black_area, _ = bottom_frame.get_areas(black_contours)
        # if black_area >= 500:
        #     stop()

        # A large enough pch of blue/orange counts as a line crossing.
        if bottom_area > 400:
            # First detection ever: lock in direction, purely by colour (blue->CCL/left,
            # orange->CWR/right). After this, the other colour is completely ignored for
            # the rest of the run.
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"
                elif bottom_colour == 2:
                    direction = "CWR"

            # Only react to a detection if it matches the locked-in colour.
            # (direction == "CCL" <-> blue, direction == "CWR" <-> orange)
            if direction == "CCL" and bottom_colour == 1:
                blue_count += 1
                print(f"BLUE: {blue_count}")
            elif direction == "CWR" and bottom_colour == 2:
                orange_count += 1
                print(f"ORANGE: {orange_count} BLUE: {blue_count}")

            # This is the last line: count it, but don't execute a turn for it -- IN_PARKING
            # takes over instead.
            if orange_count < LINE_COUNT and blue_count < LINE_COUNT:
                pending_turn = True
                matching_obstacle_seen = False  # fresh turn window -- not yet blocked by anything
                obs_cleared_time = None
                if SHOW_VID:
                    cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # else: this is the "other" colour showing up after lock-in — ignored entirely.
    # Turn execution itself now lives entirely in run_turning() -- as soon as pending_turn
    # goes True (just above), the main loop's dispatch routes to that starting next frame,
    # ahead of both WALL_FOLLOW and AVOIDING_OBSTACLE.

    # A real obstacle in view -- hand off to AVOIDING_OBSTACLE starting next frame.
    # (Same slightly-odd precedence as the original: red_area>100 OR (green_area>100 AND
    # not pending_turn), kept exactly as-is rather than "fixed" to obs_on_screen.)
    if red_area > 100 or green_area > 100 and not pending_turn:
        print(f"          %%%%%% red {red_area} green {green_area} pending turn {pending_turn}")
        state = State.AVOIDING_OBSTACLE

def run_avoiding_obstacle(gyro, cap):
    """AVOIDING_OBSTACLE's own logic: re-detects the obstacle fresh every
    frame, steers toward a pass point offset from the obstacle's own corner
    by its bounding-box height (GREEN passes left, RED passes right), and
    watches for the reverse-trigger condition -- if met, hands off to
    REVERSING starting next frame. Counted as avoided either once the
    obstacle's gone from view, or once |cam_error|/|cam_error_y| both drop
    under OBSTACLE_REACHED_PX/PY (obstacle_reached) while it's still
    visible -- i.e. the robot has actually drawn alongside its pass point,
    not just lost sight of it. Either way, hands straight back to
    WALL_FOLLOW the instant that's met, no delay. (The original's
    "was_reversing" branch is gone -- that's REVERSING's own job now that
    it's a separate state.)"""
    global steering, speed, state, obs_on_screen, obstacle_color, direction, \
           blue_count, orange_count, pending_turn, last_turn_time, \
           reversing_time, cam_error, cam_error_y, matching_obstacle_seen, \
           obs_cleared_time

    # print(f"ENTER run_avoiding_obstacle(gyro={gyro})")

    steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
    speed = 75

    # Update the middle frame and check for obstacles
    middle_frame.update(cap)
    red_contours, green_contours, yellow_contours = middle_frame.find_contours(SHOW_VID)
    red_area, _ = middle_frame.get_areas(red_contours)
    green_area, _ = middle_frame.get_areas(green_contours)
    yellow_area, _ = middle_frame.get_areas(yellow_contours)
    # Fold yellow's area into whichever colour it reacts as -- see the matching
    # comment in run_wall_follow.
    if yellow_behaviour == "RED":
        red_area += yellow_area
    elif yellow_behaviour == "GREEN":
        green_area += yellow_area

    obs_on_screen = red_area > 100 or green_area > 100

    if not pending_turn and time.time() - last_turn_time > 1.5:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours(SHOW_VID)
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours) # if bottom_colour = 1 = blue if bottom_colour = 2 = orange

        # Black flooding the bottom frame means we're too close to a wall/obstacle in
        # front -- stop immediately rather than keep driving into it.
        # black_area, _ = bottom_frame.get_areas(black_contours)
        # if black_area >= 500:
        #     stop()

        # A large enough patch of blue/orange counts as a line crossing.
        if bottom_area > 400:
            # First detection ever: lock in direction, purely by colour (blue->CCL/left,
            # orange->CWR/right). After this, the other colour is completely ignored for
            # the rest of the run.
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"
                elif bottom_colour == 2:
                    direction = "CWR"

            # Only react to a detection if it matches the locked-in colour.
            # (direction == "CCL" <-> blue, direction == "CWR" <-> orange)
            if direction == "CCL" and bottom_colour == 1:
                blue_count += 1
                print(f"BLUE: {blue_count}")
            elif  direction == "CWR" and bottom_colour == 2:
                orange_count += 1
                print(f"ORANGE: {orange_count} BLUE: {blue_count}")

            # This is the last line: count it, but don't execute a turn for it -- IN_PARKING
            # takes over instead.
            if orange_count < LINE_COUNT and blue_count < LINE_COUNT:
                pending_turn = True
                matching_obstacle_seen = False  # fresh turn window -- not yet blocked by anything
                obs_cleared_time = None
                if SHOW_VID:
                    cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # else: this is the "other" colour showing up after lock-in — ignored entirely.
    # Turn execution itself now lives entirely in run_turning() -- as soon as pending_turn
    # goes True (just above), the main loop's dispatch routes to that starting next frame,
    # ahead of both WALL_FOLLOW and AVOIDING_OBSTACLE.

    # Every contour across all colours, filtered down to real blobs (drop
    # single-pixel noise), ranked by how close its bottom edge is to the bottom of
    # the ROI (largest y = nearest the robot). The first is the closer contour, the
    # second (if any -- there's only ever 0 or 1 more) is the farther one.
    # YELLOW is only included when yellow_behaviour actually maps it to RED/GREEN --
    # "NONE" means yellow obstacles are invisible here, same as not being there at all.
    yellow_obstacle_contours = yellow_contours if yellow_behaviour in ("RED", "GREEN") else []
    all_contours = (
    [(c, "RED") for c in red_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA] +
    [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA] +
    [(c, "YELLOW") for c in yellow_obstacle_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA]
    )
    all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

    obstacle_reached = False  # true once |cam_error|/|cam_error_y| both drop under the
                               # OBSTACLE_REACHED_PX/PY thresholds -- see the unified
                               # avoided-check after this if/else.

    if all_contours:
        closest_contour, obstacle_color = all_contours[0]
        # obstacle_color is the raw detected colour (may be "YELLOW", kept for display/
        # prints); effective_color is what it's reacted to (RED/GREEN) and drives every
        # behaviour decision below.
        effective_color = effective_colour(obstacle_color)
        obstacle_area = cv2.contourArea(closest_contour)

        # Bounding box around the tracked obstacle, offset from the ROI crop into
        # full-frame coordinates.
        box_x, box_y, box_w, box_h = cv2.boundingRect(closest_contour)
        if SHOW_VID:
            cv2.rectangle(cap, (box_x + middle_frame.x1, box_y + middle_frame.y1),
                          (box_x + box_w + middle_frame.x1, box_y + box_h + middle_frame.y1), (255, 255, 255), 2)

        print(f"    ^ ^ ^ ^ ^ ^ ^ ^ ^ Saw Obstacle: {obstacle_color}")

        speed = 75

        # Bottom-left corner of the bounding box for GREEN, bottom-right for RED --
        # the pass-side corner of the box, not a point picked off the contour itself.
        if effective_color == "GREEN":
            obstacle_rel_x, obstacle_rel_y = box_x, box_y + box_h
        else:
            obstacle_rel_x, obstacle_rel_y = box_x + box_w, box_y + box_h

        # Contour coords are relative to middle_frame's ROI crop; offset to full-frame coords.
        obstacle_x = int(obstacle_rel_x) + middle_frame.x1
        obstacle_y = int(obstacle_rel_y) + middle_frame.y1

        # GREEN passes on the left, RED passes on the right -- if the close/large block
        # is already sitting on its "wrong" (already-clear) half of the frame, it isn't
        # actually boxing us in, so don't trigger a reverse for it.
        frame_mid_x = cap.shape[1] // 2
        wrong_side = (effective_color == "GREEN" and obstacle_x > frame_mid_x) or (effective_color == "RED" and obstacle_x < frame_mid_x)

        # If the obstacle's bottom edge has pushed past 3/4 of the way down the ROI and
        # a large area of it is still visible, we're too close to steer around it safely --
        # hand off to REVERSING starting next frame instead.
        if not wrong_side and obstacle_rel_y > 0.75 * (middle_frame.y2 - middle_frame.y1) and obstacle_area > 1500:
            state = State.REVERSING
            reversing_time = time.time()

        ### obstacle avoidance -- finding the point to plot and follow

        # Black ring first so the yellow fill stands out against find_contours(SHOW_VID)'s own
        # yellow contour outlines (the dot sits right on the contour edge otherwise).
        if SHOW_VID:
            cv2.circle(cap, (obstacle_x, obstacle_y), 7, (0, 0, 0), -1)      # Black outline
            cv2.circle(cap, (obstacle_x, obstacle_y), 5, (0, 255, 255), -1)  # Yellow

        # closeness: 0-1, how far down the full camera frame the obstacle's bottom edge
        # (obstacle_y) sits -- bigger obstacle_y (lower on screen) = closer to the robot.
        closeness = obstacle_y / cap.shape[0]
        closeness = max(0.0, min(1.0, closeness))  # obstacle_y can sit outside [0, frame height]

        # Desired point: a horizontal line from the obstacle's bottom-left/right corner
        # toward the pass side (GREEN -> left, RED -> right), whose length is closeness
        # scaled against a max of 3/4 the screen width -- the farther away (closeness
        # near 0), the shorter the line (point sits almost beside the obstacle); the
        # closer (closeness near 1), the longer it gets (up to 3/4 the frame width).
        # Squared rather than linear so closeness has a bigger swing near the close end
        # (where it matters most) and stays short/subtle for most of the far range.
        max_line_length = cap.shape[1] * 0.75
        pass_offset = (closeness ** 2) * max_line_length
        target_x = int(obstacle_x - pass_offset if effective_color == "GREEN" else obstacle_x + pass_offset)
        target_y = obstacle_y
        print(f"OBSTACLE DIST: obstacle_y={obstacle_y} closeness={closeness:.2f} pass_offset={pass_offset:.0f}")
        if SHOW_VID:
            cv2.line(cap, (obstacle_x, obstacle_y), (target_x, target_y), (255, 255, 0), 2)  # Cyan
            cv2.circle(cap, (target_x, target_y), 5, (255, 0, 0), -1)  # Blue

        # Robot origin = bottom-centre of the frame (where the camera/robot sits).
        heading_x = cap.shape[1] // 2
        robot_pos = (heading_x, cap.shape[0] - 1)
        if SHOW_VID:
            cv2.line(cap, robot_pos, (target_x, target_y), (0, 255, 0), 2)  # Green

        # Steer straight toward the desired point -- the camera term drives steering
        # directly here, not blended/diluted with the gyro's heading-hold term (that
        # term only knows about desired_heading, not the obstacle, and would fight
        # against actually reaching the point).
        cam_error = target_x - heading_x
        cam_error_y = robot_pos[1] - target_y
        obstacle_reached = abs(cam_error) < OBSTACLE_REACHED_PX and abs(cam_error_y) < OBSTACLE_REACHED_PY
        if SHOW_VID:
            cv2.putText(cap, f"Error: {cam_error} px (reached={obstacle_reached})", (target_x - 60, target_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Pink

        if state != State.REVERSING:
            kp = KP_OBSTACLE_RED if effective_color == "RED" else KP_OBSTACLE
            steering = int(max(45, min(135, DEFAULT_STEER_ANGLE + kp * cam_error)))

        if SHOW_VID:
            cv2.putText(cap, f"{obstacle_color} OBSTACLE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(cap, f"Speed: {speed}", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    else:
        cam_error = None
        cam_error_y = None
        steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
        speed = 75

    # Counted as avoided either when the obstacle's gone from view, or once we've closed
    # to within OBSTACLE_REACHED_PX/PY of its pass point even while it's still visible --
    # hands straight back to WALL_FOLLOW the instant that's true, no delay.
    if (not all_contours) or obstacle_reached:
        state = State.WALL_FOLLOW


def run_turning(gyro, cap):

    global steering, speed, state, obs_on_screen, obstacle_color, direction, \
           pending_turn, last_turn_time, matching_obstacle_seen, obs_cleared_time, \
           post_obs_veto_turn_start

    # print(f"ENTER run_turning(gyro={gyro})")

    steering = navigate_wall(gyro, desired_heading)  # plain wall-follow -- stays this way the whole function, no avoidance steering
    speed = 75

    middle_frame.update(cap)
    red_contours, green_contours, yellow_contours = middle_frame.find_contours(SHOW_VID)

    # YELLOW is only included when yellow_behaviour actually maps it to RED/GREEN --
    # "NONE" means yellow obstacles are invisible here, same as not being there at all.
    yellow_obstacle_contours = yellow_contours if yellow_behaviour in ("RED", "GREEN") else []
    all_contours = (
    [(c, "RED") for c in red_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA] +
    [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA] +
    [(c, "YELLOW") for c in yellow_obstacle_contours if cv2.contourArea(c) > OBSTACLE_MIN_CONTOUR_AREA]
    )
    all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

    # Same size-filtered contours as is_matching_obstacle below -- so a few stray
    # noise pixels (shadow tint, HSV bleed) can't leave obs_on_screen stuck True
    # forever after the real obstacle has left frame (that mismatch used to trap
    # the matching-colour veto below in an infinite gyro-crawl).
    obs_on_screen = bool(all_contours)

    is_matching_obstacle = False
    if all_contours:
        obstacle_color = all_contours[0][1]
        # obstacle_color is the raw detected colour (may be "YELLOW", kept for
        # display); effective_color drives every veto/turn decision below.
        effective_color = effective_colour(obstacle_color)
        is_matching_obstacle = (direction == "CCL" and effective_color == "RED") or \
                                (direction == "CWR" and effective_color == "GREEN")

    if is_matching_obstacle:
        # Matching-colour obstacle: crawl forward on gyro heading-hold alone (no camera
        # term, so it doesn't react to the obstacle/wall). No turn-safety check, nothing
        # else, until it's fully out of view (see next check below).
        matching_obstacle_seen = True
        obs_cleared_time = None  # still in view -- reset in case it flickered clear/back
        steering = gyro_only_steer(gyro, desired_heading)
        speed = MATCHING_OBSTACLE_VETO_SPEED
        if SHOW_VID:
            cv2.putText(cap, f"MATCHING OBS VETO ({obstacle_color} -- gyro crawl)", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return

    if matching_obstacle_seen and not obs_on_screen:
        # The matching-colour obstacle we just cleared -- keep gyro-crawling for
        # MATCHING_OBSTACLE_CLEAR_DELAY (a couple frames' worth) before actually firing
        # turn(), instead of turning the instant it leaves frame.
        steering = gyro_only_steer(gyro, desired_heading)
        speed = MATCHING_OBSTACLE_VETO_SPEED
        if obs_cleared_time is None:
            obs_cleared_time = time.time()  # first frame it's been fully out of view
        if time.time() - obs_cleared_time < MATCHING_OBSTACLE_CLEAR_DELAY:
            if SHOW_VID:
                cv2.putText(cap, "MATCHING OBS VETO (clearing -- gyro crawl)", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return
        # MATCHING_OBSTACLE_CLEAR_DELAY elapsed -- fire turn() immediately, no extra
        # stop-and-verify wait.
        turn(direction)
        pending_turn = False
        last_turn_time = time.time()  # cooldown before the same patch can be counted again
        state = State.POST_OBS_VETO_TURN
        post_obs_veto_turn_start = time.time()
        return

    # ---- turn execution: matching-colour veto, opposite-colour obstacle fires the turn
    # immediately rather than waiting on it. No avoidance steering here -- steering stays
    # plain wall-follow (set at the top of this function) the whole time. ----
    if direction == "CCL":  # left
        if obs_on_screen and effective_color == "RED":
            pass  # matching-colour veto: wait for RED to clear entirely
        elif obs_on_screen:
            # GREEN (opposite-colour) obstacle: fire the turn right away instead of
            # waiting for it to clear, and explicitly hand off to AVOIDING_OBSTACLE
            # so it starts dodging the block next frame, already on the new heading.
            turn(direction)
            pending_turn = False
            last_turn_time = time.time()  # cooldown before the same patch can be counted again
            state = State.AVOIDING_OBSTACLE
        else:
            # No obstacle -- fire immediately, no stop-and-verify wait.
            turn(direction)
            pending_turn = False
            last_turn_time = time.time()  # cooldown before the same patch can be counted again

    elif direction == "CWR":  # right
        if obs_on_screen and effective_color == "GREEN":
            pass  # matching-colour veto: wait for GREEN to clear entirely
        elif obs_on_screen:
            # RED (opposite-colour) obstacle: same as the CCL/GREEN case -- turn
            # immediately and explicitly hand off to AVOIDING_OBSTACLE.
            turn(direction)
            pending_turn = False
            last_turn_time = time.time()  # cooldown before the same patch can be counted again
            state = State.AVOIDING_OBSTACLE
        else:
            # No obstacle -- fire immediately, no stop-and-verify wait.
            turn(direction)
            pending_turn = False
            last_turn_time = time.time()  # cooldown before the same patch can be counted again

    # Turn done -- hand control back to whichever of WALL_FOLLOW/AVOIDING_OBSTACLE
    # actually applies next frame, same as AVOIDING_OBSTACLE's own obstacle-cleared
    # handoff. No timeout fallback -- the only way out of pending_turn/TURNING is an
    # actual turn() call.
    if not pending_turn:
        state = State.AVOIDING_OBSTACLE if obs_on_screen else State.WALL_FOLLOW


def run_post_obs_veto_turn(gyro, cap):
    """POST_OBS_VETO_TURN's own logic: entered only from run_turning's
    matching-colour-obstacle-clear path (see there), right after turn()
    already updated desired_heading. For POST_OBS_VETO_TURN_DURATION
    seconds, steers on gyro heading-hold alone (gyro_only_steer, no camera
    term) at normal driving speed, then hands off to WALL_FOLLOW -- so the
    turn coming out of the obstacle veto gets a brief beat of gyro-only
    steering instead of immediately blending in wall-follow's camera term."""
    global steering, speed, state

    # print(f"ENTER run_post_obs_veto_turn(gyro={gyro})")

    steering = gyro_only_steer(gyro, desired_heading)
    speed = 75

    if time.time() - post_obs_veto_turn_start >= POST_OBS_VETO_TURN_DURATION:
        state = State.WALL_FOLLOW


def run_reversing(gyro, cap):
    """REVERSING's own logic: back up for 1 second at a fixed steering angle,
    then hand back to AVOIDING_OBSTACLE to re-assess (matching the original
    comment's intent -- not straight back to WALL_FOLLOW). No turn-line
    detection during this second -- that's dropped here relative to the
    original (which still counted, just didn't execute, turns while reversing)."""
    global controller, steering, speed, state

    # print(f"ENTER run_reversing(gyro={gyro})")

    if time.time() - reversing_time < 1:
        state = State.REVERSING
        controller = "BWD"
        steering = 90
        speed = 75
    else:
        controller = "FWD"  # reverse window elapsed
        state = State.AVOIDING_OBSTACLE  # re-assess the obstacle next frame, don't jump straight to WALL_FOLLOW



print("ENTERING THE WHILE LOOP")


while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable -- read
                                            # here (not lower down) so OUT_PARKING can use it too

    branch = state  # snapshot before the if/elif/else below -- some branches (WALL_FOLLOW's
                    # LINE_COUNT check, OUT_PARKING's own handoff) mutate `state` mid-iteration,
                    # but SHOW_VID below must draw whatever THIS iteration actually computed,
                    # not whatever `state` happens to read as by the time we get there.

    if state == State.OUT_PARKING:
        # One-time manoeuvre, handled entirely separately from (and before) the normal
        # wall-follow/obstacle logic below: find which black wall is closer and force a
        # hard turn away from it -- max steering, opposite direction -- for 1 second.
        left_frame.update(cap)
        right_frame.update(cap)
        left_area, _ = left_frame.get_areas(left_frame.find_contours(SHOW_VID))
        right_area, _ = right_frame.get_areas(right_frame.find_contours(SHOW_VID))
        # left wall closer (bigger black area) -> steer away from it, i.e. max right; and vice versa
        out_parking_steer = 129 if left_area > right_area else 35
        print(f"OUT_PARKING: left_area={left_area:.0f} right_area={right_area:.0f} -> steer={out_parking_steer}")
        ser.write(f"{out_parking_steer},0,FWD\n".encode())
        motor_control.driveRotations(2,150,"FWD")


        # 1s elapsed (or already stopped on a previous frame): a colour matching the
        # side we just swung into (steer=40/left -> GREEN, steer=140/right -> RED) means
        # we'd be driving straight into it -- stop instead of handing off to WALL_FOLLOW.
        # Not the direction-aware is_matching_obstacle used elsewhere -- `direction`
        # isn't locked in yet this early in the run.
        middle_frame.update(cap)
        red_contours, green_contours, yellow_contours = middle_frame.find_contours(SHOW_VID)
        red_area, _ = middle_frame.get_areas(red_contours)
        green_area, _ = middle_frame.get_areas(green_contours)
        yellow_area, _ = middle_frame.get_areas(yellow_contours)
        if yellow_behaviour == "RED":
            red_area += yellow_area
        elif yellow_behaviour == "GREEN":
            green_area += yellow_area
        obs_on_screen = red_area > 100 or green_area > 100
        obs_colour = ("RED" if red_area > green_area else "GREEN") if obs_on_screen else None

        matching_obstacle = (out_parking_steer == 40 and obs_colour == "GREEN") or \
                             (out_parking_steer == 140 and obs_colour == "RED")

        if out_parking_return_turn_called:
            # Heading reverted back onto the real course -- brief gyro-only hold to let
            # it settle before handing off to WALL_FOLLOW, same settle pattern
            # POST_OBS_VETO_TURN uses after a colour-line turn.
            controller = "FWD"
            steering = gyro_only_steer(gyro, desired_heading)
            speed = 75
            if time.time() - out_parking_return_turn_start >= POST_OBS_VETO_TURN_DURATION:
                state = State.WALL_FOLLOW  # manoeuvre complete -- nothing ever sets state back to OUT_PARKING
        elif out_parking_reversed:
            # 0.5s reverse away from the obstacle is done -- turn onto a new target
            # heading (+90 if we swung right/steer=140, -90 if left/steer=40, same
            # +/-90 encoding turn() uses everywhere else) and drive forward on gyro
            # heading-hold (no camera term) toward it until that same obstacle colour
            # clears the frame entirely, then keep going straight a moment longer and
            # turn back onto the real (pre-escape) course heading.
            if not out_parking_turn_called:
                out_parking_original_heading = desired_heading  # real course heading, before the escape turn
                turn("CWR" if out_parking_steer == 140 else "CCL")
                out_parking_turn_called = True
            controller = "FWD"
            steering = gyro_only_steer(gyro, desired_heading)
            speed = 75
            still_matching = obs_on_screen and obs_colour == out_parking_obstacle_seen_colour
            if still_matching:
                out_parking_obs_clear_start = None  # obstacle back in view -- reset the clear-timer
                out_parking_continue_start = None
            else:
                # Obstacle is gone from view -- keep gyro-crawling for
                # MATCHING_OBSTACLE_CLEAR_DELAY before committing, same
                # clear-then-commit pattern run_turning uses for matching obstacles.
                if out_parking_obs_clear_start is None:
                    out_parking_obs_clear_start = time.time()
                if time.time() - out_parking_obs_clear_start >= MATCHING_OBSTACLE_CLEAR_DELAY:
                    # Clear-delay elapsed -- keep driving straight a further 0.2s, then
                    # turn back onto the real desired heading.
                    if out_parking_continue_start is None:
                        out_parking_continue_start = time.time()
                    if time.time() - out_parking_continue_start >= 0.2:
                        desired_heading = out_parking_original_heading
                        out_parking_return_turn_called = True
                        out_parking_return_turn_start = time.time()
        elif matching_obstacle or out_parking_reverse_start is not None:
            # Matching obstacle just seen (or already mid-reverse from one) -- back away
            # for 0.5s, steering opposite the turn we just made.
            if out_parking_reverse_start is None:
                out_parking_obstacle_seen_colour = obs_colour
                out_parking_reverse_start = time.time()
            controller = "BWD"
            steering = 40 if out_parking_steer == 140 else 140
            speed = 70
            if time.time() - out_parking_reverse_start >= 0.5:
                out_parking_reversed = True
        else:
            state = State.WALL_FOLLOW  # manoeuvre complete -- nothing ever sets state back to OUT_PARKING
    elif state == State.IN_PARKING:

        parking_frame.update(cap)
        red_contours, green_contours, black_contours, magenta_contours = parking_frame.find_contours(SHOW_VID)

        wall_x = None
        speed = 60  # every active driving phase below uses this; only the final hold overrides it

        if not park_square_filled:
            # Watch a fixed point (195, 58) instead of averaging a square region -- once
            # it reads black, the bay's back wall has been reached.
            black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
            if black_mask[58, 195]:
                park_square_filled = True
                park_square_filled_time = time.time()

        if not park_square_filled:
            # Gyro-only straight-line hold, against the normal desired_heading (unchanged
            # since the last line never calls turn()).
            steering = gyro_only_steer(gyro, desired_heading)
        elif not park_turned:
            if not park_turn_called:
                turn(direction)
                park_turn_called = True
            steering = gyro_only_steer(gyro, desired_heading)
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 1:
                park_turned = True
        elif not park_cleared:
            # Watch a fixed point on screen -- once it reads black, it's time for the
            # next (force) turn. (160, 35) for CCL; CWR uses (155, 35) --
            # same y on both sides, x shifts since the turn direction changes where
            # the relevant wall edge lands in frame. Not checked until a full second
            # after park_square_filled was first hit, since that point wouldn't be
            # meaningful to watch any sooner.
            if time.time() - park_square_filled_time >= 1.0:
                cleared_watch_point = (155, 40) if direction == "CWR" else (160, 40)
                watch_point_black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                if watch_point_black_mask[cleared_watch_point[1], cleared_watch_point[0]]:
                    park_cleared = True
                    park_cleared_time = time.time()

            steering = gyro_only_steer(gyro, desired_heading)
        elif STOP_AFTER_PARK_CLEARED:
            # TEMPORARY: once cleared, back away in two blocking, encoder-counted
            # driveRotations() legs (unlike the continuous driveMotor() the epilogue
            # sends every other frame) -- first 3 rotations at PWM 80 steered to the
            # hard clamp (135, same max used by the wall-tripwire deflect /
            # obstacle-avoidance clamp elsewhere), then 1 rotation at PWM 120 steered
            # to the opposite hard clamp (45) -- then hold. Each steering command is
            # sent directly here (not left to the epilogue's ser.write) so the servo
            # is actually at the clamp before driveRotations blocks the loop. Flip
            # STOP_AFTER_PARK_CLEARED back to False once the rest of the sequence
            # (force turn, final reverse, final turn) is implemented.
            steering = 45
            if not park_final_reverse_called:
                motor_control.stopMotor()
                time.sleep(1)
                ser.write(f"45,0,FWD\n".encode())
                ser.flush()
                motor_control.driveRotations(3, 120, "FWD")
                ser.write(f"82,0,BWD\n".encode())
                ser.flush()
                motor_control.driveRotations(2, 120, "BWD")
                ser.write(f"45,0,FWD\n".encode())
                ser.flush()
                motor_control.driveRotations(3,120,"BWD")
                ser.write(f"135,0,FWD\n".encode())
                ser.flush()
                motor_control.driveRotations(0.5,120,"FWD")
                ser.write("82,0,FWD\n".encode())
                park_final_reverse_called = True
            speed = 0

    elif state == State.FINISHING:
        # No longer reached from LINE_COUNT (that now enters IN_PARKING) -- left in place,
        # not deleted. One last turn() on the locked-in direction, keep driving straight on
        # that new heading for FINISHING_DURATION, then stop() once and hold.
        if not finishing_turn_called:
            turn(direction)
            finishing_turn_called = True
            finishing_start = time.time()

        if not finishing_stopped:
            steering = gyro_only_steer(gyro, desired_heading)
            speed = 75
            if time.time() - finishing_start >= FINISHING_DURATION:
                stop()
                finishing_stopped = True
                speed = 0
        else:
            steering = DEFAULT_STEER_ANGLE
            speed = 0

    elif state == State.REVERSING:
        # Let an in-progress proximity-safety backup finish its own window before
        # anything else gets a look-in, even a pending turn.
        run_reversing(gyro, cap)
    elif pending_turn:
        # TURNING takes priority over WALL_FOLLOW/AVOIDING_OBSTACLE: as soon as a
        # colour-line trips pending_turn, this branch runs instead of either of
        # those every frame until the turn actually executes -- see
        # run_turning()'s own docstring.
        state = State.TURNING
        run_turning(gyro, cap)
    elif state == State.POST_OBS_VETO_TURN:
        run_post_obs_veto_turn(gyro, cap)
    elif state == State.WALL_FOLLOW:
        run_wall_follow(gyro, cap)
    elif state == State.AVOIDING_OBSTACLE:
        run_avoiding_obstacle(gyro, cap)

    # Once either colour has been crossed LINE_COUNT times (checked regardless of which
    # of the three states above just ran, matching the original), switch to IN_PARKING
    # instead of continuing to drive. This assignment is the last word for this frame --
    # any earlier state write above is overridden here, and every subsequent frame
    # short-circuits into the IN_PARKING branch before it can reach (and re-clobber)
    # this point again.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if state != State.IN_PARKING:
            print(f"LINE_COUNT reached ({orange_count} orange / {blue_count} blue) -- entering IN_PARKING")
            state = State.IN_PARKING
            # Don't let this frame's already-computed steering (from above, before we
            # knew the last line had just been reached) reach the robot -- no turning,
            # no wall-follow steering, from this frame on.1``
            steering = DEFAULT_STEER_ANGLE
            speed = 0

    # Two-sided wall tripwire: hard-overrides steering to deflect away from whichever side's
    # point lands on black, regardless of turn direction -- a safety net independent of the
    # colour-line/obstacle logic above. Skipped during REVERSING/OUT_PARKING/FINISHING,
    # which have their own deliberate steering that this shouldn't clobber.
    if state not in (State.REVERSING, State.OUT_PARKING, State.FINISHING):
        full_hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
        full_black_mask = cv2.inRange(full_hsv, black_range[0][0], black_range[0][1])
        left_tripped = check_wall_tripwire(full_black_mask, WALL_TRIPWIRE_LEFT)
        right_tripped = check_wall_tripwire(full_black_mask, WALL_TRIPWIRE_RIGHT)
        if SHOW_VID:
            cv2.circle(cap, WALL_TRIPWIRE_LEFT, 4, (0, 0, 255) if left_tripped else (0, 255, 0), -1)
            cv2.circle(cap, WALL_TRIPWIRE_RIGHT, 4, (0, 0, 255) if right_tripped else (0, 255, 0), -1)
        if left_tripped:
            steering = WALL_TRIPWIRE_HARD_RIGHT  # deflect away from the left wall
        elif right_tripped:
            steering = WALL_TRIPWIRE_HARD_LEFT  # deflect away from the right wall

    # ---- common epilogue: single send-gate + single SHOW_VID block for every state ----y

    motor_control.driveMotor(speed * 2.54, controller)
    ser.write(f"{steering},{speed},{controller}\n".encode())  # send steering+speed to the microcontroller each loop
    # ser.write(f"{steering},0,{controller},{orange_count},open\n".encode())  # send steering+speed to the microcontroller each loop
    sent_steer = steering
    sent_speed = speed
    sent_controller = controller
    ser.flush()

    print(f"-------  ------ STATE: {state.value} Steer {sent_steer} Lines {orange_count} blines: {blue_count}\n")

    if SHOW_VID:
        # Shown every frame regardless of state/branch, unlike the branch-specific overlays below.
        cv2.putText(cap, f"direction={direction}", (10, 195),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if branch == State.OUT_PARKING:
            left_frame.draw_roi(cap)
            right_frame.draw_roi(cap)
            middle_frame.draw_roi(cap)
            cv2.putText(cap, f"obs_on_screen={obs_on_screen} color={obs_colour}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(cap, f"OUT_PARKING steer={steering} ctrl={controller}", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(cap, f"reversed={out_parking_reversed} return_turn={out_parking_return_turn_called} "
                              f"seen_colour={out_parking_obstacle_seen_colour}", (10, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            gyro_str = f"{gyro:.0f}" if gyro is not None else "None"
            cv2.putText(cap, f"Heading desired={desired_heading:.0f} current={gyro_str}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        elif branch == State.IN_PARKING:
            parking_frame.draw_roi(cap)

            cv2.putText(cap, f"obs_on_screen={obs_on_screen} color={obstacle_color}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.putText(cap, f"direction={direction}", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Watch point for park_square_filled (195, 58) -- only shown while it's still
            # relevant (before the square/point has been read as filled).
            if not park_square_filled:
                square_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[58, 195])
                cv2.circle(cap, (195, 58), 4, (0, 255, 0) if square_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, "(195,58)", (201, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if square_watch_hit else (0, 0, 255), 1)

            # Watch point for park_cleared -- (160, 35) for CCL, (155, 35) for CWR --
            # not drawn until 1s after park_square_filled_time, matching when it's
            # actually checked above.
            if not park_cleared and park_square_filled_time is not None \
                    and time.time() - park_square_filled_time >= 1.0:
                overlay_cleared_point = (155, 35) if direction == "CWR" else (160, 35)
                cleared_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                    [overlay_cleared_point[1], overlay_cleared_point[0]])
                cv2.circle(cap, overlay_cleared_point, 4, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, f"({overlay_cleared_point[0]},{overlay_cleared_point[1]})",
                            (overlay_cleared_point[0] + 6, overlay_cleared_point[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), 1)

        else:
            # ROI borders, drawn last (not in Frame.update()) so they can't bleed into
            # another Frame's crop earlier in the loop and split a straddling obstacle
            # into two contours -- see Frame.draw_roi().
            left_frame.draw_roi(cap)
            right_frame.draw_roi(cap)
            bottom_frame.draw_roi(cap)
            middle_frame.draw_roi(cap)

            # Overlay debug info (steering angle, line counts, FPS) on the preview frame.
            cv2.putText(cap, f"obs_on_screen={obs_on_screen} color={obstacle_color}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(cap, f"Steer: {steering:.2f}", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(cap, f"O: {str(orange_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,127,255), 1)
            cv2.putText(cap, f"B: {str(blue_count)}", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,127,0), 1)
            gyro_str = f"{gyro:.0f}" if gyro is not None else "None"
            cv2.putText(cap, f"Heading desired={desired_heading:.0f} current={gyro_str}", (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            frame_count += 1
            elapsed = time.time() - frame_time
            if elapsed > 1.0:
                fps = frame_count // elapsed  # rough FPS sampled once per second
                frame_count = 0
                frame_time = time.time()
            cv2.putText(cap, f"FPS: {fps:.2f}", (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # TEMPORARY DEBUG: hover-coordinate readout -- remove once done tuning ROIs/points.
        if mouse_x >= 0:
            cv2.circle(cap, (mouse_x, mouse_y), 3, (0, 255, 0), -1)
            cv2.putText(cap, f"({mouse_x},{mouse_y})", (mouse_x + 8, mouse_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        cv2.imshow("Video Frame", cap)

        time.sleep(0.01)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
            break

motor_control.stopMotor()
ser.write(f"90,0,STOP\n".encode())
ser.close()

cv2.destroyAllWindows()  