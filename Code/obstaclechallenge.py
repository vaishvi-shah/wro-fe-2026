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
import lib.motor_control as motor_control


class State(Enum):
    """Drive mode. TURNING takes priority over WALL_FOLLOW/AVOIDING_OBSTACLE
    once a colour line trips pending_turn; REVERSING still runs ahead of it."""
    OUT_PARKING = "OUT_PARKING"  # one-time manoeuvre at startup, before WALL_FOLLOW ever runs -- see out_parking_start
    WALL_FOLLOW = "WALL_FOLLOW"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    TURNING = "TURNING"  # executing a pending colour-line turn -- see pending_turn and run_turning()
    POST_OBSTACLE_TURN = "POST_OBSTACLE_TURN"  # 1s gyro-only-steer turn right after turn() fires
                                                # from the matching-obstacle-clear path in run_turning
                                                # -- see run_post_obstacle_turn()
    REVERSING = "REVERSING"
    IN_PARKING = "IN_PARKING"  # entered once LINE_COUNT is reached, instead of stopping


CALIBRATION_FILE = "lib/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present

# ToF (VL53L0X) shares the I2C bus with the IMU (IMU=0x28, ToF=0x29) -- init'd here, right
# after the bus is up. Wrapped so a missing/failed ToF doesn't take the script down.
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
sent_speed = None       # last speed actually sent
sent_controller = None  # last direction (FWD/BWD) actually sent -- a speed/direction change
                         # must resend even if steering hasn't moved >=3
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent _as 100 + this
LINE_COUNT = 12                 # number of colour-line crossings before stopping1Q
SAFE_TURN_AREA = 3000           # max black area on the side of a turn before we can safely execute the turn
MATCHING_OBSTACLE_CLEAR_DELAY = 0.4  # seconds to keep gyro-crawling past a matching-colour obstacle
                                      # clearing the frame before actually firing turn()
POST_OBSTACLE_TURN_DURATION = 2.0  # seconds POST_OBSTACLE_TURN gyro-steers on the new heading
                                    # before handing off to WALL_FOLLOW
AVOIDING_OBSTACLE_CLEAR_DELAY = 0.4  # seconds to keep waiting in AVOIDING_OBSTACLE after the
                                      # tracked obstacle clears before handing back to WALL_FOLLOW
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # camera derivative gain (damps oscillation from the wall pixel area difference)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)
KD_GYRO = 0.01  # gyro derivative gain (damps oscillation/overshoot from how fast the
                # heading error is changing), tune on track
KP_OBSTACLE = 0.4   # obstacle-avoidance proportional gain (target pixel error -> steering degrees)
KP_OBSTACLE_RED = 0.4  # RED-only override -- RED wasn't turning hard enough at the shared gain, tune on track
OBSTACLE_REACHED_PX = 10  # |cam_error| below this counts as "reached" the other-colour obstacle's pass point
OBSTACLE_REACHED_PY = 10  # |cam_error_y| below this required too -- x can align long before the robot is actually alongside the point
WALL_ROW_SEARCH_BAND = 15  # rows above/below the obstacle's own row also searched for a wall pixel -- the
                            # exact row can be occluded by the obstacle itself (e.g. block sliding in low
                            # and from the very edge), even though the wall is visible a few rows away
PARK_SQUARE_SIZE = 10  # small debug squaQre drawn during IN_PARKING, tune on track
PARK_SQUARE_CENTER_Y = 75  # tune on track
STOP_AFTER_PARK_CLEARED = True  # TEMPORARY: hold once park_cleared instead of continuing into
                                 # the force turn -- flip to False to resume the rest of IN_PARKING
# Inner-wall tripwire: points near the turn-side edge that warn if avoidance steering is
# hugging the wall too tightly, before the robot actually clips it. Tune on track.
INNER_WALL_TRIPWIRE_CWR = [(300, 190), (305, 205), (310, 220)]  # near right edge
INNER_WALL_TRIPWIRE_CCL = [(20, 190), (15, 205), (10, 220)]     # near left edge, mirror of CWR
INNER_WALL_SOFT_KP_SCALE = 0.4  # damps KP/KP_GYRO during a tripwire fallback

state = State.WALL_FOLLOW  # every branch below dispatches on this
stopping = False  # true once the stop sequence has started
reversing_time = None  # timestamp REVERSING was entered
out_parking_start = None  # timestamp the OUT_PARKING manoeuvre began; None until the first OUT_PARKING frame
out_parking_steer = None  # locked-in max-steer value (opposite the closer wall) for the OUT_PARKING manoeuvre

park_square_filled = False
park_turn_called = False
park_turned = False
park_cleared = False
park_cleared_time = None
park_force_turn_called = False   # guards the one-shot turn() call for the force-turn heading
park_force_turn_done = False     # true once gyro is within 5 deg of that target heading
park_force_turn_start = None     # timestamp the forced turn began
park_force_turn_duration = None  # how long the forced turn took, once it completes
park_force_turn_obstacle_done = False  # true once the post-force-turn obstacle check has run
park_force_turn_obstacle_reverse_start = None  # timestamp an obstacle was seen right after the force turn
park_final_reverse_start = None     # timestamp the closing reverse began
park_final_reverse_done = False     # true once the closing reverse has finished
park_final_reverse_started = False  # true once black-then-white has been seen at (151, 90)
park_final_reverse_black_seen = False  # true once that point has read black at least once
park_final_turn_called = False      # guards the one-shot turn() call for the closing turn
park_final_turn_done = False        # true once gyro is within 5 deg of the closing turn's target
park_straight_gap = None  # reference wall_x, held constant once captured after the turn

obs_on_screen = False
obstacle_color = "undefined"

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = DEFAULT_STEER_ANGLE
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)
last_turn_time = 0  # timestamp of the last executed turn; gates re-detecting the same patch of colour
matching_obstacle_seen = False  # true while run_turning is waiting out a matching-colour obstacle veto
obs_cleared_time = None  # timestamp the matching-colour obstacle first went out of view (see run_turning)
avoiding_obstacle_cleared_time = None  # timestamp the tracked obstacle first went out of view (see run_avoiding_obstacle)
post_obstacle_turn_start = None  # timestamp POST_OBSTACLE_TURN began
cam_error = None  # last known obstacle-avoidance horizontal pixel error
cam_error_y = None  # last known obstacle-avoidance vertical pixel error
last_target = None  # last known black_wall_x, reused when this frame's row lookup misses

prev_wall_error = None  # last frame's (left_area - right_area), for the KD term in navigate_wall
prev_wall_error_time = None  # timestamp prev_wall_error was captured

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")


frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()


obs_on_screen = False  # true whenever red/green obstacle area is above threshold, regardless of
                        # `state` -- purely "is an obstacle visible right now"
obs_colour = None  # "RED"/"GREEN"/None, saved alongside obs_on_screen each frame -- whichever
                    # colour has the larger area when obs_on_screen is True, else None


# defining colour ranges (HSV) used to mask each region of interest
blue_range = [
    [np.array([95, 70, 40]), np.array([145, 255, 255])]
]

orange_range = [
    [np.array([10, 40, 70]), np.array([27, 255, 255])]
]

# S spans the full range so any dark, colour-cast surface counts as black, not just true gray.
# V<=90 stays below white_range's V>=100 floor so the two don't overlap.
black_range = [
    [np.array([0, 0, 0]), np.array([180, 255, 90])]
]

red1_range = [
    [np.array([0, 120, 127]), np.array([8, 255, 255])]
]

red2_range = [
    [np.array([170, 140, 134]), np.array([180, 255, 255])]
]
red_obstacle_range = red1_range + red2_range   # one colour group, two HSV ranges (hue wraps at 0/180)

green_obstacle_range = [
    [np.array([45, 90, 60]), np.array([80, 255, 255])]
]

magenta_range = [  # parking-bay marker colour, same range as parking.py
    [np.array([150, 50, 60]), np.array([175, 255, 255])]
]

white_range = [  # low saturation, high value -- only checked during IN_PARKING's straight-reverse leg
    [np.array([0, 0, 100]), np.array([180, 90, 255])]
]


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
gyro_prev_target = None   # target_heading being tracked -- resets the above when it changes
gyro_last_derivative = 0.0  # last computed derivative, kept only for the debug overlay

def gyro_only_steer(gyro_heading, target_heading):
    """Gyro-only PD steering, no camera term -- used during IN_PARKING where the
    camera wall-following isn't reliable."""
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

    return int(max(45, min(135, DEFAULT_STEER_ANGLE - KP_GYRO * error - KD_GYRO * derivative)))

def check_inner_wall_tripwire(black_mask, direction):
    """True if any of the current direction's inner-wall tripwire points is on black.
    No direction locked in yet -> no warning."""
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
    """Blends gyro heading-hold and camera wall-balance into one steering value
    (70% gyro + 30% camera, clamped to [45, 135]). kp_scale damps both gains for
    a softer correction (e.g. the inner-wall tripwire fallback)."""
    global prev_wall_error, prev_wall_error_time

    # refresh wall-pixel area for both sides
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)


    # camera term (D term disabled below -- P only for now)
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
    steering_value = max(45, min(135, steering_value))  # clamp to servo range


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
# full frame width, not a narrow centre strip -- the obstacle drifts sideways during
# the avoidance turn and a narrow ROI was losing it before the robot had passed it

# whole-screen ROI, used once IN_PARKING starts -- replaces every other Frame above,
# since during parking nothing is confined to a known strip
parking_frame = Frame(cap, 0, 320, 0, 240, colour_range=[red_obstacle_range, green_obstacle_range,
                                                           black_range, magenta_range])

def stop():
    print("AM STOPPING")
    time.sleep(0.01)
    motor_control.stopMotor()
    ser.write(f"90,0,STOP,0,stop\n".encode())
    ser.flush()



def run_wall_follow(gyro, cap):
    """Plain wall-following steering; detects turn lines and new obstacles,
    handing off to AVOIDING_OBSTACLE starting next frame when one appears."""
    global steering, speed, state, obs_on_screen, direction, \
           blue_count, orange_count, pending_turn, last_turn_time, \
           matching_obstacle_seen, obs_cleared_time

    steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
    speed = 70

    # Update the middle frame and check for obstacles
    middle_frame.update(cap)
    red_contours, green_contours = middle_frame.find_contours()
    red_area, _ = middle_frame.get_areas(red_contours)
    green_area, _ = middle_frame.get_areas(green_contours)

    obs_on_screen = red_area > 100 or green_area > 100

    if not pending_turn and time.time() - last_turn_time > 1.5:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours()
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours)  # 1=blue, 2=orange

        # a large enough patch of blue/orange counts as a line crossing
        if bottom_area > 600:
            # first detection locks in direction; the other colour is ignored from here on
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"   # blue seen first -> locked to blue/left forever
                elif bottom_colour == 2:
                    direction = "CWR"   # orange seen first -> locked to orange/right forever

            if direction == "CCL":
                blue_count += 1
                print(f"BLUE: {blue_count}")
            else:
                orange_count += 1
                print(f"ORANGE: {orange_count}")

            # the last line is counted but not turned on -- IN_PARKING takes over instead
            if orange_count < LINE_COUNT and blue_count < LINE_COUNT:
                pending_turn = True
                matching_obstacle_seen = False  # fresh turn window -- not yet blocked by anything
                obs_cleared_time = None
                if SHOW_VID:
                    cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    # turn execution itself happens in run_turning(), dispatched once pending_turn is True

    # obstacle in view -- hand off to AVOIDING_OBSTACLE next frame
    if red_area > 100 or green_area > 100 and not pending_turn:
        state = State.AVOIDING_OBSTACLE


def run_avoiding_obstacle(gyro, cap):
    """Re-detects the obstacle every frame and steers toward the pass-point
    between it and the near wall. Hands off to REVERSING if the reverse-trigger
    condition is met, or back to WALL_FOLLOW once the obstacle is avoided
    (out of view, or within OBSTACLE_REACHED_PX/PY of the pass point) --
    waits out AVOIDING_OBSTACLE_CLEAR_DELAY before actually handing back."""
    global steering, speed, state, obs_on_screen, obstacle_color, direction, \
           blue_count, orange_count, pending_turn, last_turn_time, \
           reversing_time, last_target, cam_error, cam_error_y, matching_obstacle_seen, \
           obs_cleared_time, avoiding_obstacle_cleared_time

    steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
    speed = 70

    # full-frame black mask, reused below by the tripwire check and the wall lookup
    full_hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
    full_black_mask = cv2.inRange(full_hsv, black_range[0][0], black_range[0][1])

    middle_frame.update(cap)
    red_contours, green_contours = middle_frame.find_contours()
    red_area, _ = middle_frame.get_areas(red_contours)
    green_area, _ = middle_frame.get_areas(green_contours)

    obs_on_screen = red_area > 100 or green_area > 100

    if not pending_turn and time.time() - last_turn_time > 1.5:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours()
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours)  # 1=blue, 2=orange

        # a large enough patch of blue/orange counts as a line crossing
        if bottom_area > 600:
            # first detection locks in direction; the other colour is ignored from here on
            if not direction:
                if bottom_colour == 1:
                    direction = "CCL"   # blue seen first -> locked to blue/left forever
                elif bottom_colour == 2:
                    direction = "CWR"   # orange seen first -> locked to orange/right forever

            if direction == "CCL":
                blue_count += 1
                print(f"BLUE: {blue_count}")
            else:
                orange_count += 1
                print(f"ORANGE: {orange_count}")

            # the last line is counted but not turned on -- IN_PARKING takes over instead
            if orange_count < LINE_COUNT and blue_count < LINE_COUNT:
                pending_turn = True
                matching_obstacle_seen = False  # fresh turn window -- not yet blocked by anything
                obs_cleared_time = None
                if SHOW_VID:
                    cv2.putText(cap, f"{bottom_colour}",  (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    # turn execution itself happens in run_turning(), dispatched once pending_turn is True

    inner_wall_warning = check_inner_wall_tripwire(full_black_mask, direction)

    # debug: tripwire points, red if tripped this frame, green otherwise
    if direction == "CWR":
        tripwire_points = INNER_WALL_TRIPWIRE_CWR
    elif direction == "CCL":
        tripwire_points = INNER_WALL_TRIPWIRE_CCL
    else:
        tripwire_points = []
    for tw_x, tw_y in tripwire_points:
        tw_tripped = (0 <= tw_y < full_black_mask.shape[0] and 0 <= tw_x < full_black_mask.shape[1]
                      and full_black_mask[tw_y, tw_x])
        if SHOW_VID:
            cv2.circle(cap, (tw_x, tw_y), 4, (0, 0, 255) if tw_tripped else (0, 255, 0), -1)

    # every contour across both colours, filtered to real blobs, ranked nearest-first (largest y)
    all_contours = (
    [(c, "RED") for c in red_contours if cv2.contourArea(c) > 400] +
    [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > 400]
    )
    all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

    obstacle_reached = False  # true once |cam_error|/|cam_error_y| both drop under OBSTACLE_REACHED_PX/PY

    if all_contours:
        closest_contour, obstacle_color = all_contours[0]
        obstacle_area = cv2.contourArea(closest_contour)

        if SHOW_VID:
            area_text = f"{int(obstacle_area)}px"
            (text_w, text_h), _ = cv2.getTextSize(area_text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            cv2.putText(cap, area_text, (cap.shape[1] // 2 - text_w // 2, cap.shape[0] // 2 + text_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        box_x, box_y, box_w, box_h = cv2.boundingRect(closest_contour)
        if SHOW_VID:
            cv2.rectangle(cap, (box_x + middle_frame.x1, box_y + middle_frame.y1),
                          (box_x + box_w + middle_frame.x1, box_y + box_h + middle_frame.y1), (255, 255, 255), 2)

        print(F"OBSTACLE COLOUR: {obstacle_color}")

        speed = 70

        # pass-side corner of the box: bottom-left for GREEN, bottom-right for RED
        if obstacle_color == "GREEN":
            obstacle_rel_x, obstacle_rel_y = box_x, box_y + box_h
        else:
            obstacle_rel_x, obstacle_rel_y = box_x + box_w, box_y + box_h

        # offset from middle_frame's ROI crop into full-frame coords
        obstacle_x = int(obstacle_rel_x) + middle_frame.x1
        obstacle_y = int(obstacle_rel_y) + middle_frame.y1

        # GREEN passes on the left, RED on the right -- a block already on its clear
        # half of the frame isn't boxing us in, so don't trigger a reverse for it
        frame_mid_x = cap.shape[1] // 2
        wrong_side = (obstacle_color == "GREEN" and obstacle_x > frame_mid_x) or (obstacle_color == "RED" and obstacle_x < frame_mid_x)

        # too close to steer around safely -- hand off to REVERSING next frame
        if not wrong_side and obstacle_rel_y > 0.75 * (middle_frame.y2 - middle_frame.y1) and obstacle_area > 4000:
            state = State.REVERSING
            reversing_time = time.time()

        ### obstacle avoidance -- finding the points to plot and follow

        # closest black pixel on the pass side, searched in a row band (not just the
        # exact row) since the obstacle's own body can occlude the wall on its own row
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
                # band_xs is relative to col_lo since the mask was column-sliced -- offset back
                if obstacle_color == "GREEN":
                    black_wall_x = int(band_xs.max()) + col_lo  # right-most black pixel in the left ROI
                else:
                    black_wall_x = int(band_xs.min()) + col_lo  # left-most black pixel in the right ROI
            else:
                # no wall pixel in the band either -- fall back to the screen edge on the pass side
                black_wall_x = 0 if obstacle_color == "GREEN" else full_black_mask.shape[1] - 1

        # only the wall point falls back to its last known value on a miss --
        # obstacle_x/obstacle_y always stay live
        if black_wall_x is not None:
            last_target = black_wall_x
        elif last_target is not None:
            black_wall_x = last_target

        if SHOW_VID:
            cv2.circle(cap, (obstacle_x, obstacle_y), 7, (0, 0, 0), -1)      # Black outline
            cv2.circle(cap, (obstacle_x, obstacle_y), 5, (0, 255, 255), -1)  # Yellow

        if black_wall_x is not None:
            if SHOW_VID:
                cv2.circle(cap, (black_wall_x, obstacle_y), 7, (0, 0, 0), -1)      # Black outline
                cv2.circle(cap, (black_wall_x, obstacle_y), 5, (0, 255, 255), -1)  # Yellow
                cv2.line(cap, (obstacle_x, obstacle_y), (black_wall_x, obstacle_y), (255, 0, 255), 2)  # Pink

            # target: midpoint between the obstacle's corner point and the black wall
            target_x = (obstacle_x + black_wall_x) // 2
            target_y = obstacle_y
            if SHOW_VID:
                cv2.circle(cap, (target_x, target_y), 5, (255, 0, 0), -1)  # Blue

            # robot origin = bottom-centre of the frame
            heading_x = cap.shape[1] // 2
            robot_pos = (heading_x, cap.shape[0] - 1)
            if SHOW_VID:
                cv2.line(cap, robot_pos, (target_x, target_y), (0, 255, 0), 2)  # Green

            # steer straight at the target -- camera term drives steering directly here,
            # not blended with the gyro heading-hold term (which would fight the obstacle)
            cam_error = target_x - heading_x
            cam_error_y = robot_pos[1] - target_y
            obstacle_reached = abs(cam_error) < OBSTACLE_REACHED_PX and abs(cam_error_y) < OBSTACLE_REACHED_PY
            if SHOW_VID:
                cv2.putText(cap, f"Error: {cam_error} px (reached={obstacle_reached})", (target_x - 60, target_y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Pink

            if state != State.REVERSING:
                kp = KP_OBSTACLE_RED if obstacle_color == "RED" else KP_OBSTACLE
                steering = int(max(45, min(135, DEFAULT_STEER_ANGLE + kp * cam_error)))
                if SHOW_VID:
                    cv2.putText(cap, f"steer = {DEFAULT_STEER_ANGLE} + {kp}*{cam_error} = {steering}",
                                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        if SHOW_VID:
            cv2.putText(cap, f"{obstacle_color} OBSTACLE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(cap, f"Speed: {speed}", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    else:
        last_target = None  # obstacle cleared; don't carry a stale target into the next one
        cam_error = None
        cam_error_y = None
        steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
        speed = 70

    # avoided once out of view or within reach of the pass point -- wait out
    # AVOIDING_OBSTACLE_CLEAR_DELAY before handing back to WALL_FOLLOW
    if (not all_contours) or obstacle_reached:
        if avoiding_obstacle_cleared_time is None:
            avoiding_obstacle_cleared_time = time.time()  # first frame counted as avoided
        if time.time() - avoiding_obstacle_cleared_time >= AVOIDING_OBSTACLE_CLEAR_DELAY:
            state = State.WALL_FOLLOW
            avoiding_obstacle_cleared_time = None
    else:
        avoiding_obstacle_cleared_time = None  # still avoiding it -- reset in case it flickered


def run_turning(gyro, cap):
    """Executes a pending colour-line turn once it's safe to. Dispatched ahead of
    WALL_FOLLOW/AVOIDING_OBSTACLE every frame while pending_turn is True. A
    matching-colour obstacle (RED for CCL, GREEN for CWR) vetoes the turn and
    crawls forward on gyro heading-hold until it clears, then hands off to
    POST_OBSTACLE_TURN. An opposite-colour obstacle does not block the turn --
    it fires immediately and hands off to AVOIDING_OBSTACLE instead. No timeout
    fallback: the only way out of pending_turn/TURNING is an actual turn() call."""
    global steering, speed, state, obs_on_screen, obstacle_color, direction, \
           pending_turn, last_turn_time, matching_obstacle_seen, post_obstacle_turn_start, \
           obs_cleared_time

    steering = navigate_wall(gyro, desired_heading)  # plain wall-follow, no avoidance steering
    speed = 70

    # used below for the inner-wall tripwire debug overlay
    full_hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
    full_black_mask = cv2.inRange(full_hsv, black_range[0][0], black_range[0][1])

    middle_frame.update(cap)
    red_contours, green_contours = middle_frame.find_contours()
    red_area, _ = middle_frame.get_areas(red_contours)
    green_area, _ = middle_frame.get_areas(green_contours)

    obs_on_screen = red_area > 100 or green_area > 100

    all_contours = (
    [(c, "RED") for c in red_contours if cv2.contourArea(c) > 400] +
    [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > 400]
    )
    all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

    is_matching_obstacle = False
    if all_contours:
        obstacle_color = all_contours[0][1]
        is_matching_obstacle = (direction == "CCL" and obstacle_color == "RED") or \
                                (direction == "CWR" and obstacle_color == "GREEN")

    if is_matching_obstacle:
        # matching-colour obstacle: crawl forward on gyro heading-hold alone until it clears
        matching_obstacle_seen = True
        obs_cleared_time = None  # still in view -- reset in case it flickered clear/back
        steering = gyro_only_steer(gyro, desired_heading)
        speed = 70
        if SHOW_VID:
            cv2.putText(cap, f"MATCHING OBS VETO ({obstacle_color} -- gyro crawl)", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return

    if matching_obstacle_seen and not obs_on_screen:
        # obstacle just cleared -- keep gyro-crawling for MATCHING_OBSTACLE_CLEAR_DELAY
        # before actually firing turn()
        steering = gyro_only_steer(gyro, desired_heading)
        speed = 70
        if obs_cleared_time is None:
            obs_cleared_time = time.time()  # first frame it's been fully out of view
        if time.time() - obs_cleared_time < MATCHING_OBSTACLE_CLEAR_DELAY:
            if SHOW_VID:
                cv2.putText(cap, "MATCHING OBS VETO (clearing -- gyro crawl)", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return
        # delay elapsed -- fire the turn and hand off to POST_OBSTACLE_TURN (gyro-only)
        turn(direction)
        pending_turn = False
        last_turn_time = time.time()  # cooldown before the same patch can be counted again
        state = State.POST_OBSTACLE_TURN
        post_obstacle_turn_start = time.time()
        return

    if direction == "CWR":
        tripwire_points = INNER_WALL_TRIPWIRE_CWR
    elif direction == "CCL":
        tripwire_points = INNER_WALL_TRIPWIRE_CCL
    else:
        tripwire_points = []
    for tw_x, tw_y in tripwire_points:
        tw_tripped = (0 <= tw_y < full_black_mask.shape[0] and 0 <= tw_x < full_black_mask.shape[1]
                      and full_black_mask[tw_y, tw_x])
        if SHOW_VID:
            cv2.circle(cap, (tw_x, tw_y), 4, (0, 0, 255) if tw_tripped else (0, 255, 0), -1)

    # ---- turn execution: safe-to-turn wall check + matching-colour veto ----
    left_frame.update(cap)
    right_frame.update(cap)

    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()

    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    # checked to make sure that it is safe to turn (i.e. not too close to a wall)
    if direction == "CCL":  # left
        if left_area < SAFE_TURN_AREA: # black area on left is small enough to turn left
            if obs_on_screen and obstacle_color == "RED":
                pass  # matching-colour veto: wait for RED to clear entirely
            elif obs_on_screen:
                # opposite-colour (GREEN) obstacle: fire the turn immediately and
                # hand off to AVOIDING_OBSTACLE
                turn(direction)
                pending_turn = False
                last_turn_time = time.time()  # cooldown before the same patch can be counted again
                state = State.AVOIDING_OBSTACLE
            else:
                turn(direction)
                pending_turn = False
                last_turn_time = time.time()  # cooldown before the same patch can be counted again

    elif direction == "CWR":  # right
        if right_area < SAFE_TURN_AREA: # black area on right is small enough to turn right
            if obs_on_screen and obstacle_color == "GREEN":
                pass  # matching-colour veto: wait for GREEN to clear entirely
            elif obs_on_screen:
                # opposite-colour (RED) obstacle: fire the turn immediately and
                # hand off to AVOIDING_OBSTACLE
                turn(direction)
                pending_turn = False
                last_turn_time = time.time()  # cooldown before the same patch can be counted again
                state = State.AVOIDING_OBSTACLE
            else:
                turn(direction)
                pending_turn = False
                last_turn_time = time.time()  # cooldown before the same patch can be counted again

    # turn done -- hand control back to whichever of WALL_FOLLOW/AVOIDING_OBSTACLE applies
    if not pending_turn:
        state = State.AVOIDING_OBSTACLE if obs_on_screen else State.WALL_FOLLOW


def run_post_obstacle_turn(gyro, cap):
    """Entered from run_turning's matching-colour-obstacle-clear path. Steers on
    gyro heading-hold alone for POST_OBSTACLE_TURN_DURATION seconds, then hands
    off to WALL_FOLLOW."""
    global steering, speed, state

    steering = gyro_only_steer(gyro, desired_heading)
    speed = 70

    if time.time() - post_obstacle_turn_start >= POST_OBSTACLE_TURN_DURATION:
        state = State.WALL_FOLLOW


def run_reversing(gyro, cap):
    """Backs up for 1 second at a fixed steering angle, then hands back to
    AVOIDING_OBSTACLE to re-assess. No turn-line detection during this second."""
    global controller, steering, speed, state

    if time.time() - reversing_time < 1:
        state = State.REVERSING
        controller = "BWD"
        steering = 90
        speed = 70
    else:
        controller = "FWD"  # reverse window elapsed
        state = State.AVOIDING_OBSTACLE  # re-assess the obstacle next frame, don't jump straight to WALL_FOLLOW


print("ENTERING THE WHILE LOOP")


while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable

    branch = state  # snapshot before the if/elif/else below, since some branches mutate
                    # `state` mid-iteration but SHOW_VID must draw what this iteration computed

    if state == State.OUT_PARKING:
        # one-time manoeuvre: find which black wall is closer and force a hard turn
        # away from it -- max steering, opposite direction -- for 1 second
        left_frame.update(cap)
        right_frame.update(cap)
        left_area, _ = left_frame.get_areas(left_frame.find_contours())
        right_area, _ = right_frame.get_areas(right_frame.find_contours())

        if out_parking_start is None:
            out_parking_start = time.time()
            # left wall closer (bigger black area) -> steer away from it, i.e. max right; and vice versa
            out_parking_steer = 130 if left_area > right_area else 50
            print(f"OUT_PARKING: left_area={left_area:.0f} right_area={right_area:.0f} -> steer={out_parking_steer}")

        steering = out_parking_steer
        speed = 70

        if time.time() - out_parking_start >= 1.0:
            # min turn duration met -- hold the turn until any obstacle on the side we
            # turned into (same colour convention as the pending_turn veto) clears
            middle_frame.update(cap)
            red_contours, green_contours = middle_frame.find_contours()
            red_area, _ = middle_frame.get_areas(red_contours)
            green_area, _ = middle_frame.get_areas(green_contours)
            obs_on_screen = red_area > 100 or green_area > 100
            obstacle_color = "RED" if red_area >= green_area else "GREEN"

            if (out_parking_steer == 130 and red_area > 100) or (out_parking_steer == 50 and green_area > 100):
                print(f"OUT_PARKING: holding turn, {obstacle_color} obstacle still visible "
                      f"(red={red_area:.0f} green={green_area:.0f})")
            else:
                state = State.WALL_FOLLOW  # manoeuvre complete -- nothing ever sets state back to OUT_PARKING
    elif state == State.IN_PARKING:

        parking_frame.update(cap)
        red_contours, green_contours, black_contours, magenta_contours = parking_frame.find_contours()

        wall_x = None
        speed = 70  # every active driving phase below uses this; only the final hold overrides it

        if not park_square_filled:
            # watch a fixed point (195, 82) -- once it reads black, the bay's back wall is reached
            black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
            if black_mask[82, 195]:
                park_square_filled = True

        if not park_square_filled:
            steering = gyro_only_steer(gyro, desired_heading)
        elif not park_turned:
            if not park_turn_called:
                turn(direction)
                park_turn_called = True
            steering = gyro_only_steer(gyro, desired_heading)
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_turned = True
        elif not park_cleared:
            # watch point for the next (force) turn -- (160, 62) for CCL, (155, 30) for CWR,
            # since the turn direction shifts where the relevant wall edge lands in frame
            cleared_watch_point = (155, 30) if direction == "CWR" else (160, 62)
            watch_point_black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
            if watch_point_black_mask[cleared_watch_point[1], cleared_watch_point[0]]:
                park_cleared = True
                park_cleared_time = time.time()

            steering = gyro_only_steer(gyro, desired_heading)
        elif STOP_AFTER_PARK_CLEARED:
            # TEMPORARY: hold here once cleared instead of continuing into the force turn.
            # Flip STOP_AFTER_PARK_CLEARED back to False to resume the rest of the sequence.
            steering = DEFAULT_STEER_ANGLE
            speed = 0
        elif not park_force_turn_done:
            # forced max turn away from the pink marker just passed (130=CWR/right, 50=CCL/left),
            # runs until the gyro shows a full 90 deg of rotation
            if not park_force_turn_called:
                turn(direction)
                park_force_turn_called = True
                park_force_turn_start = time.time()
            steering = 130 if direction == "CWR" else 50
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_force_turn_done = True
                park_force_turn_duration = time.time() - park_force_turn_start
                print(f"Force turn took {park_force_turn_duration:.3f}s")
 
        elif not park_final_reverse_done:
            # confirm desired_heading is still met (momentum/overshoot after the force
            # turn can leave it slightly off) before committing to the straight reverse
            heading_confirmed = gyro is not None and abs(angle_error(gyro, desired_heading)) < 5
            if not heading_confirmed:
                controller = "BWD"
                # gyro_only_steer() is tuned for forward motion -- mirror its output
                # around 90 since the same steering angle yaws the opposite way in reverse
                steering = 2 * DEFAULT_STEER_ANGLE - gyro_only_steer(gyro, desired_heading)
                steering = max(45, min(135, steering))
            else:
                # whole-frame white detection, only used during this straight-reverse leg
                white_mask = cv2.inRange(parking_frame.hsv, white_range[0][0], white_range[0][1])
                white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                if white_contours and SHOW_VID:
                    cv2.drawContours(parking_frame.frame, white_contours, -1, (255, 255, 255), 1)

                if not park_final_reverse_started:
                    # hold here until (151, 90) has gone black then white -- white alone
                    # doesn't trigger the reverse until black has been seen there first
                    steering = DEFAULT_STEER_ANGLE
                    speed = 0
                    black_at_point = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[90, 151]
                    if black_at_point:
                        park_final_reverse_black_seen = True
                    elif park_final_reverse_black_seen and white_mask[90, 151]:
                        park_final_reverse_started = True
                else:
                    # reverse straight (steering locked at 90) -- ToF trigger disabled for now, see below
                    # tof_distance = get_tof_distance()
                    # if tof_distance is not None and tof_distance < 400:
                    #     park_final_reverse_done = True
                    #     controller = "FWD"
                    if park_final_reverse_start is None:
                        park_final_reverse_start = time.time()
                    controller = "BWD"
                    steering = DEFAULT_STEER_ANGLE

                    # for now: reverse until there's no black left at watch point (151, 122)
                    final_reverse_black_mask = cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                    if not final_reverse_black_mask[122, 151]:
                        park_final_reverse_done = True
                        controller = "FWD"
        elif not park_final_turn_done:
            # forced max turn, reversing while turning -- unwinds the force-turn (opposite
            # direction), runs until the gyro shows another 90 deg of rotation
            if not park_final_turn_called:
                turn("CCL")
                park_final_turn_called = True
            controller = "BWD"
            steering = 130
            if gyro is not None and abs(angle_error(gyro, desired_heading)) < 5:
                park_final_turn_done = True
                controller = "FWD"
        else:
            # final turn complete -- hold here for now
            steering = DEFAULT_STEER_ANGLE
            speed = 0

    elif state == State.REVERSING:
        # let an in-progress proximity-safety backup finish before anything else, even a pending turn
        run_reversing(gyro, cap)
    elif pending_turn:
        # TURNING takes priority over WALL_FOLLOW/AVOIDING_OBSTACLE once a colour line trips it
        state = State.TURNING
        run_turning(gyro, cap)
    elif state == State.POST_OBSTACLE_TURN:
        run_post_obstacle_turn(gyro, cap)
    elif state == State.WALL_FOLLOW:
        run_wall_follow(gyro, cap)
    elif state == State.AVOIDING_OBSTACLE:
        run_avoiding_obstacle(gyro, cap)

    # once either colour has been crossed LINE_COUNT times, switch to IN_PARKING --
    # this overrides any state written above, and every later frame short-circuits
    # into the IN_PARKING branch at the top of the loop
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if state != State.IN_PARKING:
            print(f"LINE_COUNT reached ({orange_count} orange / {blue_count} blue) -- entering IN_PARKING")
            state = State.IN_PARKING
            # drop this frame's already-computed steering -- no turning/wall-follow from here on
            steering = DEFAULT_STEER_ANGLE
            speed = 0

    # ---- common epilogue: single send-gate + single SHOW_VID block for every state ----

    if abs(sent_steer - steering) >= 3 or speed != sent_speed or controller != sent_controller:
        motor_control.driveMotor(speed * 2.54, controller)
        ser.write(f"{steering-5},{speed},{controller},{orange_count},open\n".encode())  # send steering+speed to the microcontroller each loop
        # ser.write(f"{steering-5},0,{controller},{orange_count},open\n".encode())  # send steering+speed to the microcontroller each loop
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

            # gyro_only_steer() breakdown, shown whenever it's the active steering source.
            # Reads the already-computed `steering`/`gyro_last_derivative` instead of
            # calling gyro_only_steer() again -- it's stateful, a second call would corrupt it.
            reverse_heading_unconfirmed = (park_force_turn_done and not park_final_reverse_done
                                            and gyro is not None
                                            and abs(angle_error(gyro, desired_heading)) >= 5)
            gyro_only_steer_active = (not park_force_turn_done) or reverse_heading_unconfirmed
            if gyro is not None and gyro_only_steer_active:
                gyro_error = angle_error(gyro, desired_heading)
                cv2.putText(cap,
                            f"gyro_only_steer={DEFAULT_STEER_ANGLE}-{KP_GYRO}*{gyro_error:.1f}-{KD_GYRO}*{gyro_last_derivative:.1f}={steering}",
                            (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1)

            # ToF reading, shown for reference (not the active trigger right now)
            if park_force_turn_done and not park_final_reverse_done:
                cv2.putText(cap, f"tof_distance={get_tof_distance()} mm", (10, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # watch point (151, 90): reverse holds here until black-then-white.
            # red=no black yet, yellow=black seen, green=white seen (about to trigger)
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

            # watch point (151, 122): reverse continues until this reads no black
            if park_force_turn_done and not park_final_reverse_done and park_final_reverse_started:
                final_reverse_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[122, 151])
                cv2.circle(cap, (151, 122), 4, (0, 255, 0) if final_reverse_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, "(151,122)", (157, 126),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if final_reverse_watch_hit else (0, 0, 255), 1)

            # watch point for park_square_filled (195, 82)
            if not park_square_filled:
                square_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])[82, 195])
                cv2.circle(cap, (195, 82), 4, (0, 255, 0) if square_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, "(195,82)", (201, 86),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if square_watch_hit else (0, 0, 255), 1)

            # watch point for park_cleared -- (160, 62) for CCL, (155, 30) for CWR
            if not park_cleared:
                overlay_cleared_point = (155, 30) if direction == "CWR" else (160, 62)
                cleared_watch_hit = bool(
                    cv2.inRange(parking_frame.hsv, black_range[0][0], black_range[0][1])
                    [overlay_cleared_point[1], overlay_cleared_point[0]])
                cv2.circle(cap, overlay_cleared_point, 4, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), -1)
                cv2.putText(cap, f"({overlay_cleared_point[0]},{overlay_cleared_point[1]})",
                            (overlay_cleared_point[0] + 6, overlay_cleared_point[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0) if cleared_watch_hit else (0, 0, 255), 1)

            # bounding rect around each pink marker (should be two of them)
            for c in magenta_contours:
                if cv2.contourArea(c) > 50:
                    mx, my, mw, mh = cv2.boundingRect(c)
                    cv2.rectangle(cap, (mx, my), (mx + mw, my + mh), (255, 0, 255), 2)

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
            # ROI borders, drawn last so they can't bleed into another Frame's crop earlier
            # in the loop and split a straddling obstacle into two contours
            left_frame.draw_roi(cap)
            right_frame.draw_roi(cap)
            bottom_frame.draw_roi(cap)
            middle_frame.draw_roi(cap)

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
    print(f"STATE: {state.value}")

    time.sleep(0.01)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        stop()
        ser.flush()
        break

motor_control.stopMotor()
ser.write(f"90,0,STOP,0,stop\n".encode())
ser.close()


cv2.destroyAllWindows()
