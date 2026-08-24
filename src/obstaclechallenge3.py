# 1 ----------------------------------------------------------------------------------------
"""
make the parking square size and location the sam I REMOVED THE THE 3 DEGREE THING -- CHECK CHECK CHECK
increased all speed by 7
"""


# imports!
import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055
import board
import adafruit_vl53l0x
from enum import Enum


class State(Enum):
    """Drive-mode only. TURNING is intentionally not a member: it's an
    independent cooldown (see `turning`/`turning_time`), not a drive mode --
    it can be active at the same time as WALL_FOLLOW or AVOIDING_OBSTACLE, and
    folding it into this exclusive state caused the cooldown to be clobbered
    every frame by the obstacle-detection branch below."""
    OUT_PARKING = "OUT_PARKING"  # one-time manoeuvre at startup, before WALL_FOLLOW ever runs -- see out_parking_start
    WALL_FOLLOW = "WALL_FOLLOW"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    REVERSING = "REVERSING"
    IN_PARKING = "IN_PARKING"  # entered once LINE_COUNT is reached, instead of stopping


CALIBRATION_FILE = "lib/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present

# ToF (VL53L0X) shares the same I2C bus as the IMU (IMU=0x28, ToF=0x29, see tof_tester.py) --
# activated right here alongside it, not lazily later, since board.I2C() must be grabbed
# once the bus is already up. Wrapped so a missing/failed ToF doesn't take the whole
# script down -- tof stays None and get_tof_distance() below just reports unavailable.
tof = None
try:
    tof_i2c = board.I2C()
    tof = adafruit_vl53l0x.VL53L0X(tof_i2c)
    print(f"INFO: ToF (VL53L0X) Initialized. Distance: {tof.range} mm")
except Exception as tof_e:
    print(f"WARNING: ToF (VL53L0X) initialisation failed: {tof_e}")
    tof = None


def get_tof_distance():
    """Returns the current ToF range in mm, or None if unavailable."""
    if tof:
        try:
            return tof.range
        except Exception:
            return None
    return None


controller = "FWD"
sent_steer = 0
sent_speed = None    # last speed actually sent -- a phase/state transition that changes speed or
sent_controller = None  # controller (e.g. FWD->BWD) without also moving steering by >=3 must still
                        # resend, or the robot keeps executing whatever was last physically sent
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent _as 100 + this
LINE_COUNT = 1                # number of colour-line crossings before stopping1Q
SAFE_TURN_AREA = 2000           # max black area on the side of a turn before we can safely execute the turn
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # camera derivative gain (damps oscillation from the wall pixel area difference)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)
KD_GYRO = 0.01  # gyro derivative gain (damps oscillation/overshoot from how fast the
                # heading error is changing), tune on track
KP_OBSTACLE = 0.4   # obstacle-avoidance proportional gain (target pixel error -> steering degrees)
KP_OBSTACLE_RED = 0.4  # RED-only override -- RED wasn't turning hard enough at the shared gain, tune on track
OBSTACLE_REACHED_PX = 20  # |cam_error| below this counts as "reached" the other-colour obstacle's pass point
OBSTACLE_REACHED_PY = 30  # |cam_error_y| below this required too -- x can align long before the robot is actually alongside the point
WALL_ROW_SEARCH_BAND = 15  # rows above/below the obstacle's own row also searched for a wall pixel -- the
                            # exact row can be occluded by the obstacle itself (e.g. block sliding in low
                            # and from the very edge), even though the wall is visible a few rows away
PARK_SQUARE_SIZE = 10  # small debug square drawn during IN_PARKING, tune on track
PARK_SQUARE_CENTER_Y = 75  # tune on track
# Inner-wall tripwire: 3 points per turn direction, staggered diagonally toward the
# bottom-inner corner of the frame (closest to the robot, on the side it'll pass on
# after the locked-in turn). If obstacle-avoidance steering hugs that side tightly
# enough to put one of these points on black, that's a frame's warning before the
# robot actually clips the wall. CWR/CCL sets are mirror images of each other. Tune on track.
INNER_WALL_TRIPWIRE_CWR = [(300, 190), (305, 205), (310, 220)]  # near right edge, staggered down-right
INNER_WALL_TRIPWIRE_CCL = [(20, 190), (15, 205), (10, 220)]     # near left edge, mirror of CWR
INNER_WALL_SOFT_KP_SCALE = 0.4  # damps KP/KP_GYRO during a tripwire fallback -- soft peel-off, not a sharp correction

state = State.WALL_FOLLOW  # authoritative: every branch below dispatches on this. Starts here so the
                            # out-parking manoeuvre runs before anything else; nothing ever sets state
                            # back to OUT_PARKING once it leaves, so it's guaranteed one-time.
stopping = False    # true once the stop sequence has started -- independent of `state`, which
                    # gets reassigned every frame by the obstacle-detection branch and would
                    # otherwise reset stop_time on every iteration (see State docstring re: `turning`)
reversing_time = None  # timestamp REVERSING was entered
out_parking_start = None  # timestamp the OUT_PARKING manoeuvre began; None until the first OUT_PARKING frame
out_parking_steer = None  # locked-in max-steer value (opposite the closer wall) for the OUT_PARKING manoeuvre

park_square_filled = False
park_turn_called = False
park_turned = False
park_cleared = False
park_cleared_time = None
park_force_turn_called = False  # guards the one-shot turn(direction) call that sets the
                                 # force-turn's target heading (desired_heading +/- 90)
park_force_turn_done = False    # true once gyro reads within 5 deg of that target heading
park_force_turn_start = None    # timestamp the forced turn began -- tune the 5 deg gyro
                                 # threshold against this once park_force_turn_duration is known
park_force_turn_duration = None # how long the forced turn actually took, once it completes
park_force_turn_obstacle_done = False  # true once the post-force-turn obstacle check has run (and,
                                        # if an obstacle was seen, the 1s reverse away from it is done)
park_force_turn_obstacle_reverse_start = None  # timestamp a red/green obstacle was seen right after the
                                                # force turn -- not None while backing away from it for 1s
                                                # (opposite lock from the force turn's own steering)
park_final_reverse_start = None     # timestamp the closing 1.5s reverse began
park_final_reverse_done = False     # true once that 1.5s has elapsed
park_final_reverse_started = False  # true once black-then-white has been seen at (151, 90) --
                                     # gates the straight reverse itself, held before that
park_final_reverse_black_seen = False  # true once that same point has read black at least
                                        # once -- white only counts as the start trigger after
                                        # this, so a point that's white from the start doesn't
                                        # immediately (falsely) trigger the reverse
park_final_turn_called = False      # guards the one-shot turn(direction) call that adds
                                     # another 90 to desired_heading for the closing turn
park_final_turn_done = False        # true once gyro reads within 5 deg of that target heading
park_straight_gap = None  # reference wall_x, captured the first time it's visible after the
                          # turn; steering then holds the wall at this same x

obs_on_screen = False
obstacle_color = "undefined"

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = DEFAULT_STEER_ANGLE
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)
last_turn_time = 0  # timestamp of the last executed turn; gates re-detecting the same patch of colour
cam_error = None  # last known obstacle-avoidance horizontal pixel error; None until the avoidance block first sets it
cam_error_y = None  # last known obstacle-avoidance vertical pixel error (robot vs. target_y); same lifecycle as cam_error
last_target = None  # last known black_wall_x, reused only when this frame's row lookup misses

prev_wall_error = None  # last frame's (left_area - right_area), for the KD term in navigate_wall
prev_wall_error_time = None  # timestamp prev_wall_error was captured

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")


frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()


turning = False              # true while inside the post-detection "turn window" -- independent of `state`
turning_time = time.time()  # timestamp of the last colour-line detection

obs_on_screen = False  # true whenever red/green obstacle area is above threshold, regardless of
                        # `turning`/`state` -- purely "is an obstacle visible right now"
obs_colour = None  # "RED"/"GREEN"/None, saved alongside obs_on_screen each frame -- whichever
                    # colour has the larger area when obs_on_screen is True, else None


# defining colour ranges (HSV) used to mask each region of interest
blue_range = [
    [np.array([95, 70, 40]), np.array([145, 255, 255])]
]

orange_range = [
    [np.array([10, 40, 70]), np.array([27, 255, 255])]
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
    [np.array([0, 120, 107]), np.array([8, 255, 255])]
]

red2_range = [
    [np.array([170, 140, 114]), np.array([180, 255, 255])]
]
red_obstacle_range = red1_range + red2_range   # one colour group, two HSV ranges (hue wraps at 0/180)

green_obstacle_range = [
    [np.array([45, 90, 60]), np.array([80, 255, 255])]
]

magenta_range = [  # parking-bay marker colour, same range as parking.py
    [np.array([150, 50, 60]), np.array([175, 255, 255])]
]

white_range = [  # low saturation, high value -- only checked whole-frame during the
    # straight-reverse leg of IN_PARKING, not part of parking_frame's usual groups.
    # Widened from V>=200/S<=60 to also catch dimmer/shadowed and slightly-tinted white,
    # then again to V>=100 so light grey tones count too.
    [np.array([0, 0, 100]), np.array([180, 90, 255])]
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
CAM_WEIGHT = 0.5


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

    return int(max(55, min(125, DEFAULT_STEER_ANGLE - KP_GYRO * error - KD_GYRO * derivative)))

def check_inner_wall_tripwire(black_mask, direction):
    """
    Returns True if any of the current turn direction's inner-wall tripwire points
    (see INNER_WALL_TRIPWIRE_CWR/CCL) lands on black in this frame's mask. With no
    direction locked in yet there's no "inner wall" side to check, so that case is
    treated as no warning rather than guessed at.
    """
    if direction == "CWR":
        points = INNER_WALL_TRIPWIRE_CWR
    elif direction == "CCL":
        points = INNER_WALL_TRIPWIRE_CCL
    else:
        return False

    h, w = black_mask.shape[:2]
    for x, y in points:
        if 0 <= y < h and 0 <= x < w and black_mask[y, x]:
            return True
    return False


def navigate_wall(gyro_heading, desired_heading=0, kp_scale=1.0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro_heading vs desired_heading).
      2. Camera term: proportional correction on left/right wall pixel area difference (original logic).
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [55, 125] (90 +/- 35).

    `kp_scale` damps both KP and KP_GYRO below their tuned defaults -- used for a
    softer, more cautious correction (e.g. the inner-wall tripwire fallback) without
    touching the normal wall-follow gains.
    """
    global prev_wall_error, prev_wall_error_time

    # Refresh the side frames with the latest camera capture and re-run the
    # colour mask + contour detection so we know how much "wall" each side sees.
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)


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
    steering_value = max(55, min(125, steering_value))  # clamp to servo range


    gyro_heading_str = f"{gyro_heading:.0f}" if gyro_heading is not None else "None"
    print(f"gyro heading: {gyro_heading_str}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, steer: {steering_value:.0f}")
    # cv2.putText(cap, f"gyro={DEFAULT_STEER_ANGLE}-{KP_GYRO}*{kp_scale}*{heading_error:.1f}={gyro_steer:.0f}",
    #             (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    # cv2.putText(cap, f"cam={DEFAULT_STEER_ANGLE}+{KP}*{kp_scale}*{wall_error:.0f}+{KD}*{kp_scale}*{wall_error_deriv:.0f}={cam_steer:.0f}",
    #             (10, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    # cv2.putText(cap, f"steer={GYRO_WEIGHT}*gyro+{CAM_WEIGHT}*cam={steering_value:.0f}",
    #             (10, 166), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)


    return int(steering_value)


# execution of main program
cv2.startWindowThread()  # needed so cv2.imshow updates without blocking on this thread

# TEMPORARY DEBUG: hover-coordinate readout on the preview window -- remove once done tuning ROIs/points.
mouse_x, mouse_y = -1, -1

def _on_mouse_move(event, x, y, flags, param):
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y

cv2.namedWindow("Video Frame")
cv2.setMouseCallback("Video Frame", _on_mouse_move)


# initializing the camera
print("-- INITIALIZING CAMERA --")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()
cap = picam2.capture_array("main")  # grab one frame to size the ROI frames below


# initializing frames: each Frame watches a fixed region of interest (ROI) for a colour mask.
# left/right strips watch for the black wall; bottom strip watches for blue/orange turn markers.
left_frame = Frame(cap, 0, 80, 60, 200, colour_range=[black_range])
right_frame = Frame(cap, 240, 320, 60, 200, colour_range=[black_range])
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])
middle_frame = Frame(cap, 0, 320, 40, 220, colour_range=[red_obstacle_range, green_obstacle_range])
# Full frame width, not a narrow centre strip: during the avoidance turn the obstacle
# drifts sideways in-frame, and a narrow ROI was clipping/losing it well before the robot
# had actually passed it.

# Whole-screen ROI, only used once IN_PARKING starts -- replaces every other (specialised,
# strip-watching) Frame above for the rest of the run: watches the entire frame at once for
# red, green, black, and magenta, since during parking nothing is confined to a known strip.
parking_frame = Frame(cap, 0, 320, 0, 240, colour_range=[red_obstacle_range, green_obstacle_range,
                                                           black_range, magenta_range])

def stop():
    print("AM STOPPING")
    time.sleep(0.01)
    ser.write(f"90,0,STOP,0,stop\n".encode())
    ser.flush()



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
        left_area, _ = left_frame.get_areas(left_frame.find_contours())
        right_area, _ = right_frame.get_areas(right_frame.find_contours())

        if out_parking_start is None:
            out_parking_start = time.time()
            # left wall closer (bigger black area) -> steer away from it, i.e. max right; and vice versa
            out_parking_steer = 125 if left_area > right_area else 55
            print(f"OUT_PARKING: left_area={left_area:.0f} right_area={right_area:.0f} -> steer={out_parking_steer}")

        steering = out_parking_steer
        speed = 70

        if time.time() - out_parking_start >= 1.0:
            # Minimum turn duration met -- before completing the manoeuvre, check for the
            # matching-colour obstacle on the side we just turned into (RED when we turned
            # right/away-from-left-wall, GREEN when we turned left/away-from-right-wall,
            # same convention as the pending_turn veto below). Keep holding the turn until
            # it clears instead of finishing straight into it.
            middle_frame.update(cap)
            red_contours, green_contours = middle_frame.find_contours()
            red_area, _ = middle_frame.get_areas(red_contours)
            green_area, _ = middle_frame.get_areas(green_contours)
            obs_on_screen = red_area > 100 or green_area > 100
            obstacle_color = "RED" if red_area >= green_area else "GREEN"

            if (out_parking_steer == 135 and red_area > 100) or (out_parking_steer == 45 and green_area > 100):
                print(f"OUT_PARKING: holding turn, {obstacle_color} obstacle still visible "
                      f"(red={red_area:.0f} green={green_area:.0f})")
            else:
                state = State.WALL_FOLLOW  # manoeuvre complete -- nothing ever sets state back to OUT_PARKING
    elif state == State.IN_PARKING:

        parking_frame.update(cap)
        red_contours, green_contours, black_contours, magenta_contours = parking_frame.find_contours()

        wall_x = None
        speed = 60  # every active driving phase below uses this; only the final hold overrides it

        if not park_square_filled:
            # Watch a fixed point (195, 82) instead Qof averaging a square region -- once
            # it reads black, the bay's back wall has been reached.
            black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
            if black_mask[82, 195]:
                park_square_filled = True

        if not park_square_filled:
            # Gyro-only straight-line hold, against the normal desired_heading (unchanged
            # since the last line never calls turn()).
            steering = gyro_only_steer(gyro, desired_heading)
        elif not park_turned:
            if not park_turn_called:
                turn(direction)
                park_turn_called = True
            steering = gyro_only_steer(gyro, desired_heading)
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_turned = True
        elif not park_cleared:
            # Watch a fixed point on screen -- once it reads black, it's time for the
            # next (force) turn. (160, 62) for CCL; CWR uses (155, 52)
            # instead, since the turn direction shifts where the relevant wall edge
            # lands in frame.
            cleared_watch_point = (155, 52) if direction == "CWR" else (160, 62)
            watch_point_black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
            if watch_point_black_mask[cleared_watch_point[1], cleared_watch_point[0]]:
                park_cleared = True
                park_cleared_time = time.time()

            steering = gyro_only_steer(gyro, desired_heading)
        elif not park_force_turn_done:
            # Forced max turn away from the pink marker just passed (125=CWR/right,
            # 55=CCL/left). Runs until the gyro shows a full 90 degrees of rotation
            # from here, tracked via desired_heading (same turn() used for the
            # initial bay-entry turn), uninterrupted -- obstacle check happens only
            # once this completes, below.
            if not park_force_turn_called:
                turn(direction)
                park_force_turn_called = True
                park_force_turn_start = time.time()
            steering = 125 if direction == "CWR" else 55
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_force_turn_done = True
                park_force_turn_duration = time.time() - park_force_turn_start
                print(f"Force turn took {park_force_turn_duration:.3f}s")
        # -- park_force_turn_obstacle_done still disabled for now (see below).
        # elif not park_force_turn_obstacle_done:
        #     # Force turn just finished -- check once for a red/green obstacle and, if
        #     # one's visible, reverse away from it for 1s at the opposite steering lock
        #     # from the force turn's own, before continuing into the closing reverse.
        #     red_area, _ = parking_frame.get_areas(red_contours)
        #     green_area, _ = parking_frame.get_areas(green_contours)
        #     obstacle_seen = red_area > 100 or green_area > 100
        #
        #     if park_force_turn_obstacle_reverse_start is not None:
        #         # Already backing away -- finish out the full 1s regardless of
        #         # whether the obstacle is still visible.
        #         controller = "BWD"
        #         steering = 55 if direction == "CWR" else 125
        #         speed = 60
        #         if time.time() - park_force_turn_obstacle_reverse_start >= 1.0:
        #             park_force_turn_obstacle_done = True
        #             controller = "FWD"
        #     elif obstacle_seen:
        #         park_force_turn_obstacle_reverse_start = time.time()
        #         controller = "BWD"
        #         steering = 55 if direction == "CWR" else 125
        #         speed = 60
        #     else:
        #         # Nothing in the way -- nothing to do, move straight on.
        #         park_force_turn_obstacle_done = True
        #         steering = DEFAULT_STEER_ANGLE
        #         speed = 60
        elif not park_final_reverse_done:
            # Before committing to the straight-locked reverse, confirm desired_heading
            # is actually still met (momentum/overshoot right after the force turn can
            # leave it slightly off) -- hold here correcting via the gyro, still reversing
            # (not forward), until it's back within 5 deg, same threshold the turns use.
            heading_confirmed = gyro is not None and abs(angle_error(gyro, desired_heading)) < 5
            if not heading_confirmed:
                controller = "BWD"
                # gyro_only_steer() is tuned for forward motion -- the same steering
                # angle yaws the chassis the opposite way in reverse, so mirror its
                # output around 90 (same compensation park_final_turn_done makes by
                # inverting the target instead, since that phase uses a pinned value).
                steering = 2 * DEFAULT_STEER_ANGLE - gyro_only_steer(gyro, desired_heading)
                steering = max(55, min(125, steering))
            else:
                # Whole-frame white detection -- only checked during this straight-reverse
                # leg, not one of parking_frame's normal always-on groups.
                white_mask = cv2.inRange(parking_frame.hsv, white_range[0][0], white_range[0][1])
                white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                if white_contours:
                    cv2.drawContours(parking_frame.frame, white_contours, -1, (255, 255, 255), 1)

                if not park_final_reverse_started:
                    # Hold here (heading confirmed, not yet reversing) until (151, 90) has
                    # gone black then white -- white alone doesn't trigger the reverse until
                    # black has been seen there first, so a point that's already white at
                    # the start of this phase doesn't falsely trigger it immediately.
                    steering = DEFAULT_STEER_ANGLE
                    speed = 0
                    black_at_point = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[90, 151]
                    if black_at_point:
                        park_final_reverse_black_seen = True
                    elif park_final_reverse_black_seen and white_mask[90, 151]:
                        park_final_reverse_started = True
                else:
                    # Reverse straight (steering locked at 90, not gyro-corrected) until the
                    # ToF reads less than 450mm behind us -- no camera point for this one,
                    # the ToF is the trigger.
                    # -- ToF trigger disabled for now, see below.
                    # tof_distance = get_tof_distance()
                    # if tof_distance is not None and tof_distance < 400:
                    #     park_final_reverse_done = True
                    #     controller = "FWD"
                    if park_final_reverse_start is None:
                        park_final_reverse_start = time.time()
                    controller = "BWD"
                    steering = DEFAULT_STEER_ANGLE

                    # For now: reverse until there's no black left at watch point (151, 122)
                    # instead of the ToF trigger above.
                    final_reverse_black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                    if not final_reverse_black_mask[122, 151]:
                        park_final_reverse_done = True
                        controller = "FWD"
        elif not park_final_turn_done:
            # Forced max turn, reversing while turning -- runs until the gyro shows
            # another 90 degrees of rotation from here. Inverted vs. every other turn()
            # call in this maneuver: this closing turn swings back the opposite way
            # (unwinding the force-turn just before it), which for CWR means turn("CCL")
            # at steering=125. CCL now always mirrors CWR's own final turn here (same
            # turn() call and steering) instead of mirroring itself.
            if not park_final_turn_called:
                turn("CCL")
                park_final_turn_called = True
            controller = "BWD"
            steering = 125
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_final_turn_done = True
                controller = "FWD"
        else:
            # Final turn complete -- just hold here for now (park_force_turn_obstacle_done
            # phase and the "fully complete, stop+break" ending are still disabled above).
            steering = DEFAULT_STEER_ANGLE
            speed = 0

        # Once both pink markers have been seen, every phase's own speed choice is
        # overridden to a flat 70 -- except the final "fully parked, stopped" branch
        # above, which stays at 0 so the robot actually stops at the end.

    else:
        steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
        speed =87

        # Full-frame black/white mask -- built once per frame here and reused by both the
        # inner-wall tripwire check below and the obstacle pass-point wall lookup further
        # down, instead of recomputing the same cvtColor+inRange twice per frame.
        full_hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
        full_black_mask = cv2.inRange(full_hsv, black_range[0][0], black_range[0][1])

        # Update the middle frame and check for obstacles
        middle_frame.update(cap)
        red_contours, green_contours = middle_frame.find_contours()
        red_area, _ = middle_frame.get_areas(red_contours)
        green_area, _ = middle_frame.get_areas(green_contours)

        obs_on_screen = red_area > 100 or green_area > 100

        if not pending_turn and time.time() - last_turn_time > 1.5:
            bottom_frame.update(cap)
            blue_contours, orange_contours = bottom_frame.find_contours()
            bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours) # if bottom_colour = 1 = blue if bottom_colour = 2 = orange
            # print("GOT THE AREAS")


            # A large enough patch of blue/orange counts as a line crossing.
            if bottom_area > 600:
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
                if direction == "CCL":
                    blue_count += 1
                    print(f"BLUE: {blue_count}")
                else:
                    orange_count += 1
                    print(f"ORANGE: {orange_count}")

                # This is the last line: count it, but don't execute a turn for it -- IN_PARKING
                # takes over instead (triggered below, after the obstacle-avoidance block).
                if orange_count < LINE_COUNT and blue_count < LINE_COUNT:
                    pending_turn = True
                    turning_time = time.time()  # start of this turn window (stuck-turn fallback below)
                    cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                # else: this is the "other" colour showing up after lock-in — ignored entirely.
        elif pending_turn:
            # Debug: mark end of turn windowq
            if time.time() - turning_time > 1.5:
                pending_turn = False


        if pending_turn and state is not State.REVERSING: # if a turn is pending and no block is currently being avoided, execute it
            left_frame.update(cap)
            right_frame.update(cap)


            left_contours = left_frame.find_contours()
            right_contours = right_frame.find_contours()


            left_area, _ = left_frame.get_areas(left_contours)
            right_area, _ = right_frame.get_areas(right_contours)


            # checked to make sure that it is safe to turn (i.e. not too close to a wall)
            if direction == "CCL":  # left
                if left_area < SAFE_TURN_AREA: # black area on left is small enough to turn left
                    if obs_on_screen and obstacle_color == "GREEN":
                        pass  # matching-colour veto: wait for GREEN to clear entirely, same as before
                    elif obs_on_screen:
                        # other-colour (RED) obstacle: keep steering toward its avoidance point
                        # (handled below) and only pull the trigger on the turn once we've
                        # actually reached it -- both x and y close, not just x, instead of
                        # jumping the heading immediately.
                        if (cam_error is not None and abs(cam_error) < OBSTACLE_REACHED_PX
                                and cam_error_y is not None and abs(cam_error_y) < OBSTACLE_REACHED_PY):
                            turn(direction)
                            pending_turn = False
                            last_turn_time = time.time()  # cooldown before the same patch can be counted again
                    else:
                        turn(direction)
                        pending_turn = False
                        last_turn_time = time.time()  # cooldown before the same patch can be counted again

            elif direction == "CWR":  # right
                if right_area < SAFE_TURN_AREA: # black area on right is small enough to turn right
                    if obs_on_screen and obstacle_color == "RED":
                        pass  # matching-colour veto: wait for RED to clear entirely, same as before
                    elif obs_on_screen:
                        # other-colour (GREEN) obstacle: keep steering toward its avoidance point
                        # (handled below) and only pull the trigger on the turn once we've
                        # actually reached it -- both x and y close, not just x, instead of
                        # jumping the heading immediately.
                        if (cam_error is not None and abs(cam_error) < OBSTACLE_REACHED_PX
                                and cam_error_y is not None and abs(cam_error_y) < OBSTACLE_REACHED_PY):
                            turn(direction)
                            pending_turn = False
                            last_turn_time = time.time()  # cooldown before the same patch can be counted again
                    else:
                        turn(direction)
                        pending_turn = False
                        last_turn_time = time.time()  # cooldown before the same patch can be counted again

        # Inner-wall tripwire, checked every frame (not just while avoiding an obstacle) and
        # before the red/green obstacle handling below decides how to steer, so that handling
        # can see this frame's warning in time to react to it.
        inner_wall_warning = check_inner_wall_tripwire(full_black_mask, direction)

        # Debug: draw the current direction's tripwire points -- red if that specific point
        # is on black (tripped) this frame, green otherwise. Nothing to draw until direction
        # locks in, matching check_inner_wall_tripwire's own "no direction yet" behaviour.
        if direction == "CWR":
            tripwire_points = INNER_WALL_TRIPWIRE_CWR
        elif direction == "CCL":
            tripwire_points = INNER_WALL_TRIPWIRE_CCL
        else:
            tripwire_points = []
        for tw_x, tw_y in tripwire_points:
            tw_tripped = (0 <= tw_y < full_black_mask.shape[0] and 0 <= tw_x < full_black_mask.shape[1]
                          and full_black_mask[tw_y, tw_x])
            cv2.circle(cap, (tw_x, tw_y), 4, (0, 0, 255) if tw_tripped else (0, 255, 0), -1)

        was_reversing = (state == State.REVERSING)  # captured before the state overwrite below -- must run
                                                      # unconditionally since all_contours (below) can be
                                                      # non-empty even when this frame's area gate is False
        if red_area > 100 or green_area > 100 and not pending_turn:
            state = State.AVOIDING_OBSTACLE

            print(f"Red area: {red_area}, Green area: {green_area}")

            # Every contour across both colours, filtered down to real blobs (drop
            # single-pixel noise), ranked by how close its bottom edge is to the bottom of
            # the ROI (largest y = nearest the robot). The first is the closer contour, the
            # second (if any -- there's only ever 0 or 1 more) is the farther one.
        all_contours = (
        [(c, "RED") for c in red_contours if cv2.contourArea(c) > 400] +
        [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > 400]
        )
        all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

        if all_contours:
            closest_contour, obstacle_color = all_contours[0]
            obstacle_area = cv2.contourArea(closest_contour)

            area_text = f"{int(obstacle_area)}px"
            (text_w, text_h), _ = cv2.getTextSize(area_text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            cv2.putText(cap, area_text, (cap.shape[1] // 2 - text_w // 2, cap.shape[0] // 2 + text_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            # Bounding box around the tracked obstacle, offset from the ROI crop into
            # full-frame coordinates.
            box_x, box_y, box_w, box_h = cv2.boundingRect(closest_contour)
            cv2.rectangle(cap, (box_x + middle_frame.x1, box_y + middle_frame.y1),
                          (box_x + box_w + middle_frame.x1, box_y + box_h + middle_frame.y1), (255, 255, 255), 2)

            print(F"OBSTACLE COLOUR: {obstacle_color}")

            # The obstacle's pixel area grows as the robot gets closer (and shrinks again
            # once most of it drops out of the ROI at the very last moment), so use it
            # directly as the "how close/how much of it is visible" proximity measure --
            # slow down as more of the obstacle comes into view, giving more time to avoid it.
            proximity = (obstacle_area - 400) / (4000 - 400)  # 400 = detection threshold, 4000 = "close" tune on track
            proximity = max(0.0, min(1.0, proximity))
            speed = int(87 - proximity * (87 - 72))  # 87 far away, 72 once close -- tune on track

            # Bottom-left corner of the bounding box for GREEN, bottom-right for RED --
            # the pass-side corner of the box, not a point picked off the contour itself.
            if obstacle_color == "GREEN":
                obstacle_rel_x, obstacle_rel_y = box_x, box_y + box_h
            else:
                obstacle_rel_x, obstacle_rel_y = box_x + box_w, box_y + box_h

            # Contour coords are relative to middle_frame's ROI crop; offset to full-frame coords.
            obstacle_x = int(obstacle_rel_x) + middle_frame.x1
            obstacle_y = int(obstacle_rel_y) + middle_frame.y1

            # If the obstacle's bottom edge has pushed past 3/4 of the way down the ROI and
            # a large area of it is still visible, we're too close to steer around it safely --
            # reverse instead. Set steering back to straight and flip the controller field to
            # the reverse signal before backing up.


    # ---- reversing logc ------

            # GREEN passes on the left, RED passes on the right -- if the close/large block
            # is already sitting on its "wrong" (already-clear) half of the frame, it isn't
            # actually boxing us in, so don't trigger a reverse for it.
            frame_mid_x = cap.shape[1] // 2
            wrong_side = (obstacle_color == "GREEN" and obstacle_x > frame_mid_x) or (obstacle_color == "RED" and obstacle_x < frame_mid_x)

            if not was_reversing:
                if not wrong_side and obstacle_rel_y > 0.75 * (middle_frame.y2 - middle_frame.y1) and obstacle_area > 4000:
                    state = State.REVERSING
                    reversing_time = time.time()
            else:
                if time.time() - reversing_time < 1:
                    state = State.REVERSING
                    controller = "BWD"
                    steering = 90
                    speed = 90
                else:
                    controller = "FWD"  # reverse window elapsed, state already AVOIDING_OBSTACLE above

            if False:  # inner_wall_warning and state == State.AVOIDING_OBSTACLE:
                # Too close to the inner wall to trust the obstacle pass-point this frame --
                # skip the obstacle-angle calculation entirely below and peel off along the
                # wall instead, with gains damped so it's a soft correction, not a sharp one.
                # -- DISABLED: inner-wall peel-off no longer overrides obstacle-avoidance steering.
                steering = navigate_wall(gyro, desired_heading, kp_scale=INNER_WALL_SOFT_KP_SCALE)
            else:
         ### obstacle avoidance -- finding the points to plot and follow

                # Look for the closest black pixel on the pass side in a small vertical band
                # around the obstacle's own row -- restricted to left_frame's/right_frame's own
                # ROI columns, since those are the actual wall-watching strips. A band instead of
                # just the exact row: when the obstacle is sitting low and right at the frame edge
                # (e.g. slid in horizontally from outside), its own body can fully occlude the wall
                # on that exact row even though the wall is clearly visible a few rows away.
                black_wall_x = None
                if 0 <= obstacle_y < full_black_mask.shape[0]:
                    row_lo = max(0, obstacle_y - WALL_ROW_SEARCH_BAND)
                    row_hi = min(full_black_mask.shape[0], obstacle_y + WALL_ROW_SEARCH_BAND + 1)
                    if obstacle_color == "GREEN":
                        col_lo, col_hi = left_frame.x1, left_frame.x2
                    else:
                        col_lo, col_hi = right_frame.x1, right_frame.x2
                    _, band_xs = np.nonzero(full_black_mask[row_lo:row_hi, col_lo:col_hi])
                    if band_xs.size > 0:
                        # band_xs is relative to col_lo since the mask was column-sliced -- offset back.
                        if obstacle_color == "GREEN":
                            black_wall_x = int(band_xs.max()) + col_lo  # right-most black pixel in the left ROI
                        else:
                            black_wall_x = int(band_xs.min()) + col_lo  # left-most black pixel in the right ROI
                    else:
                        # No wall pixel anywhere in the band either -- most likely the wall itself is
                        # out of frame at this point (e.g. obstacle right up against the very edge).
                        # Fall back to the screen edge on the pass side rather than leaving black_wall_x
                        # unset, so the pass point/line still draw instead of vanishing.
                        black_wall_x = 0 if obstacle_color == "GREEN" else full_black_mask.shape[1] - 1

                # The wall search above only checks the obstacle's exact current row, so a single
                # noisy frame can miss it even though the obstacle itself was found fine. Only the
                # wall point falls back to its last known value — obstacle_x/obstacle_y always stay
                # live so the plotted points keep tracking the robot's actual motion instead of
                # freezing every time the wall lookup whiffs on one row.
                if black_wall_x is not None:
                    last_target = black_wall_x
                elif last_target is not None:
                    black_wall_x = last_target

                # Black ring first so the yellow fill stands out against find_contours()'s own
                # yellow contour outlines (the dot sits right on the contour edge otherwise).
                cv2.circle(cap, (obstacle_x, obstacle_y), 7, (0, 0, 0), -1)      # Black outline
                cv2.circle(cap, (obstacle_x, obstacle_y), 5, (0, 255, 255), -1)  # Yellow

                if black_wall_x is not None:
                    cv2.circle(cap, (black_wall_x, obstacle_y), 7, (0, 0, 0), -1)      # Black outline
                    cv2.circle(cap, (black_wall_x, obstacle_y), 5, (0, 255, 255), -1)  # Yellow
                    cv2.line(cap, (obstacle_x, obstacle_y), (black_wall_x, obstacle_y), (255, 0, 255), 2)  # Pink

                    # Desired point: the midpoint between the obstacle's corner point and the black wall.
                    target_x = (obstacle_x + black_wall_x) // 2
                    target_y = obstacle_y
                    cv2.circle(cap, (target_x, target_y), 5, (255, 0, 0), -1)  # Blue

                    # Robot origin = bottom-centre of the frame (where the camera/robot sits).
                    heading_x = cap.shape[1] // 2
                    robot_pos = (heading_x, cap.shape[0] - 1)
                    cv2.line(cap, robot_pos, (target_x, target_y), (0, 255, 0), 2)  # Green

                    # Steer straight toward the desired point -- the camera term drives steering
                    # directly here, not blended/diluted with the gyro's heading-hold term (that
                    # term only knows about desired_heading, not the obstacle, and would fight
                    # against actually reaching the point).
                    cam_error = target_x - heading_x
                    cam_error_y = robot_pos[1] - target_y
                    cv2.putText(cap, f"Error: {cam_error} px", (target_x - 60, target_y - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Pink

                    if state != State.REVERSING:
                        kp = KP_OBSTACLE_RED if obstacle_color == "RED" else KP_OBSTACLE
                        steering = int(max(55, min(125, DEFAULT_STEER_ANGLE + kp * cam_error)))
                        cv2.putText(cap, f"steer = {DEFAULT_STEER_ANGLE} + {kp}*{cam_error} = {steering}",
                                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            cv2.putText(cap, f"{obstacle_color} OBSTACLE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(cap, f"Speed: {speed}", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            last_target = None  # obstacle cleared; don't carry a stale target into the next one
            cam_error = None
            cam_error_y = None
            state = State.WALL_FOLLOW  # no block in view; may be upgraded to TURNING/IN_PARKING below
            steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
            speed = 87


        # Only look for a new turn-colour line if we're outside the "just turned" cooldown window.
        # Independent of `state`/obstacle-avoidance on purpose: an obstacle mid-turn-window should
        # still get avoided, and the cooldown must hold regardless of drive mode.


        # Once either colour has been crossed LINE_C                                 OUNT times (the last line, already counted
        # above without a turn), switch to IN_PARKING. This assignment is the last word for this
        # frame -- any earlier state write above (e.g. the no-obstacle branch's WALL_FOLLOW) is
        # overridden here, and every subsequent frame short-circuits into the IN_PARKING branch
        # at the top of the loop before it can reach (and re-clobber) this point again.
        if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
            if state != State.IN_PARKING:
                print(f"LINE_COUNT reached ({orange_count} orange / {blue_count} blue) -- entering IN_PARKING")
                state = State.IN_PARKING
                # Don't let this frame's already-computed navigate_wall/turn steering (from
                # above, before we knew the last line had just been reached) reach the robot --
                # no turning, no wall-follow steering, from this frame on.
                steering = DEFAULT_STEER_ANGLE
                speed = 0

    # ---- common epilogue: single send-gate + single SHOW_VID block for every state ----

    if abs(sent_steer - steering) >= 3 or speed != sent_speed or controller != sent_controller:
        ser.write(f"{steering-5},{speed},{controller},{orange_count},open\n".encode())  # send steering+speed to the microcontroller each loop
        sent_steer = steering
        sent_speed = speed
        sent_controller = controller
        ser.flush()

    if SHOW_VID:
        if branch == State.OUT_PARKING:
            left_frame.draw_roi(cap)
            right_frame.draw_roi(cap)
            cv2.putText(cap, f"obs_on_screen={obs_on_screen} color={obstacle_color}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(cap, f"OUT_PARKING steer={steering}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            gyro_str = f"{gyro:.0f}" if gyro is not None else "None"
            cv2.putText(cap, f"Heading desired={desired_heading:.0f} current={gyro_str}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        elif branch == State.IN_PARKING:
            parking_frame.draw_roi(cap)

            cv2.putText(cap, f"obs_on_screen={obs_on_screen} color={obstacle_color}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.putText(cap, f"direction={direction}", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # gyro_only_steer() calculation breakdown, shown whenever it's the active
            # steering source (park_square_filled wait, initial turn, cleared-watch, or
            # the heading-confirmation hold before the final reverse -- not the pinned-
            # max-steer force/final turns, and not the reverse itself, which locks
            # straight at 90 once heading's confirmed). Reads the already-computed
            # `steering`/`gyro_last_derivative` rather than calling gyro_only_steer()
            # again here -- it's stateful (updates the prev-error/time used for the
            # derivative as a side effect), so a second call would corrupt it.
            reverse_heading_unconfirmed = (park_force_turn_done and not park_final_reverse_done
                                            and gyro is not None
                                            and abs(angle_error(gyro, desired_heading)) >= 5)
            gyro_only_steer_active = (not park_force_turn_done) or reverse_heading_unconfirmed
            if gyro is not None and gyro_only_steer_active:
                gyro_error = angle_error(gyro, desired_heading)
                cv2.putText(cap,
                            f"gyro_only_steer={DEFAULT_STEER_ANGLE}-{KP_GYRO}*{gyro_error:.1f}-{KD_GYRO}*{gyro_last_derivative:.1f}={steering}",
                            (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1)

            # ToF reading, shown only while it's actually driving the final-reverse
            # trigger (once the force turn is done, before the reverse is done).
            # -- not the active trigger right now (see final-reverse black-point watch
            # below), still shown for reference.
            if park_force_turn_done and not park_final_reverse_done:
                cv2.putText(cap, f"tof_distance={get_tof_distance()} mm", (10, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Watch point that starts the final reverse -- (151, 90) -- reverse holds here
            # (speed 0) until this point has gone black then white. Red = hasn't seen black
            # yet, yellow = black seen, waiting on white, green = white seen (about to trigger).
            if park_force_turn_done and not park_final_reverse_done and not park_final_reverse_started:
                final_reverse_start_black_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[90, 151])
                final_reverse_start_white_hit = bool(
                    cv2.inRange(parking_frame.hsv, white_range[0][0], white_range[0][1])[90, 151])
                if park_final_reverse_black_seen and final_reverse_start_white_hit:
                    watch_colour = (0, 255, 0)
                elif park_final_reverse_black_seen or final_reverse_start_black_hit:
                    watch_colour = (0, 255, 255)
                else:
                    watch_colour = (0, 0, 255)
                cv2.circle(cap, (151, 90), 4, watch_colour, -1)
                cv2.putText(cap, "(151,90)", (157, 94),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, watch_colour, 1)

            # Watch point for the final reverse -- (151, 122) -- reverse continues until
            # this reads no black.
            if park_force_turn_done and not park_final_reverse_done and park_final_reverse_started:
                final_reverse_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[122, 151])
                cv2.circle(cap, (151, 122), 4, (0, 255, 0) if final_reverse_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, "(151,122)", (157, 126),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if final_reverse_watch_hit else (0, 0, 255), 1)

            # Watch point for park_square_filled (195, 82) -- only shown while it's still
            # relevant (before the square/point has been read as filled).
            if not park_square_filled:
                square_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[82, 195])
                cv2.circle(cap, (195, 82), 4, (0, 255, 0) if square_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, "(195,82)", (201, 86),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if square_watch_hit else (0, 0, 255), 1)

            # Watch point for park_cleared -- (160, 62) for CCL, (155, 52) for CWR --
            # shown for the whole run, not just once park_turned, so it's visible from
            # the start instead of appearing late.
            if not park_cleared:
                overlay_cleared_point = (155, 52) if direction == "CWR" else (160, 62)
                cleared_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                    [overlay_cleared_point[1], overlay_cleared_point[0]])
                cv2.circle(cap, overlay_cleared_point, 4, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, f"({overlay_cleared_point[0]},{overlay_cleared_point[1]})",
                            (overlay_cleared_point[0] + 6, overlay_cleared_point[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), 1)

            # Bounding rect around each pink marker (should be two of them).
            for c in magenta_contours:
                if cv2.contourArea(c) > 50:
                    mx, my, mw, mh = cv2.boundingRect(c)
                    cv2.rectangle(cap, (mx, my), (mx + mw, my + mh), (255, 0, 255), 2)

            # Wall point tracked for the keep-straight steering above, and the reference
            # x it's being held against.
            if wall_x is not None:
                cv2.circle(cap, (wall_x, PARK_SQUARE_CENTER_Y), 6, (0, 0, 0), -1)
                cv2.circle(cap, (wall_x, PARK_SQUARE_CENTER_Y), 4, (0, 255, 255), -1)
                if park_straight_gap is not None:
                    cv2.line(cap, (park_straight_gap, PARK_SQUARE_CENTER_Y - 15),
                              (park_straight_gap, PARK_SQUARE_CENTER_Y + 15), (0, 255, 0), 1)
                cv2.putText(cap, f"wall_x={wall_x}", (wall_x + 8, PARK_SQUARE_CENTER_Y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            red_area, _ = parking_frame.get_areas(red_contours)
            green_area, _ = parking_frame.get_areas(green_contours)
            black_area, _ = parking_frame.get_areas(black_contours)
            magenta_area, _ = parking_frame.get_areas(magenta_contours)
            cv2.putText(cap, f"PARKING R={red_area:.0f} G={green_area:.0f} B={black_area:.0f} M={magenta_area:.0f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            gyro_str = f"{gyro:.0f}" if gyro is not None else "None"
            cv2.putText(cap, f"Heading desired={desired_heading:.0f} current={gyro_str}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            magenta_count = len([c for c in magenta_contours if cv2.contourArea(c) > 200])
            cv2.putText(cap, f"magenta_count={magenta_count} cleared={park_cleared}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(cap, f"straight_gap={park_straight_gap}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(cap, f"force_turn_duration={park_force_turn_duration}", (10, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

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

    print(steering)
    print(f"STATE: {state.value}{' +TURNING' if turning else ''}")

    time.sleep(0.01)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        stop()
        ser.flush()
        break


ser.write(f"90,0,STOP,0,stop\n".encode())
ser.close()


cv2.destroyAllWindows()
