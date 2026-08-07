# 1 ----------------------------------------------------------------------------------------
"""
Obstacle avoidance has been extended to anticipate multiple obstacles. In
addition to calculating the pass point for the nearest obstacle, the algorithm
also determines the pass point for a second visible obstacle. As the robot
approaches the first obstacle, the steering target is gradually blended toward
the second obstacle's pass point using a proximity measure based on both
contour area and vertical image position. This predictive approach produces
smoother steering transitions and reduces abrupt direction changes when
navigating consecutive obstacles.
"""


# imports!
import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055
from enum import Enum


class State(Enum):
    """Drive-mode only. TURNING is intentionally not a member: it's an
    independent cooldown (see `turning`/`turning_time`), not a drive mode --
    it can be active at the same time as WALL_FOLLOW or AVOIDING_OBSTACLE, and
    folding it into this exclusive state caused the cooldown to be clobbered
    every frame by the obstacle-detection branch below."""
    PARKING = "PARKING"
    WALL_FOLLOW = "WALL_FOLLOW"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    REVERSING = "REVERSING"
    STOPPED = "STOPPED"


CALIBRATION_FILE = "lib/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present
controller = "FWD"
sent_steer = 0
SHOW_VID = True                 # toggle live OpenCV preview window
draw = True                     # toggle whether debug overlays are drawn onto the preview frame
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent _as 100 + this
LINE_COUNT = 12                # number of colour-line crossings before stopping1Q
SAFE_TURN_AREA = 2000           # max black area on the side of a turn before we can safely execute the turn
PARKING_MAGENTA_EXIT_AREA = 300  # magenta area below this counts as "marker cleared" -- ok to exit PARKING
PARKING_MIN_TURN_TIME = 0.5     # minimum forced-turn duration before the magenta exit check kicks in
PARKING_MAX_TURN_TIME = 3.0     # hard cap on the forced turn, in case the marker never clears
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)
KP_OBSTACLE = 0.4   # obstacle-avoidance proportional gain (target pixel error -> steering degrees)
state = State.PARKING  # authoritative: every branch below dispatches on this -- car starts parked
stop_time = None    # timestamp STOPPED was entered; set the first frame state becomes STOPPED
stopping = False    # true once the stop sequence has started -- independent of `state`, which
                    # gets reassigned every frame by the obstacle-detection branch and would
                    # otherwise reset stop_time on every iteration (see State docstring re: `turning`)
reversing_time = None  # timestamp REVERSING was entered
parking_turn_time = time.time()  # timestamp the forced parking-exit turn started (state starts in PARKING)




ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = DEFAULT_STEER_ANGLE
speed = 0
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)
last_target = None  # last known black_wall_x, reused only when this frame's row lookup misses

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")


frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()


turning = False              # true while inside the post-detection "turn window" -- independent of `state`
turning_time = time.time()  # timestamp of the last colour-line detection


# defining colour ranges (HSV) used to mask each region of interest
blue_range = [
    [np.array([105, 140, 80]), np.array([135, 255, 255])]
]

orange_range = [
    [np.array([14, 60, 100]), np.array([25, 255, 255])]
]

# Capped at S=90: a dark pixel only counts as "black" if it's also
# desaturated (true gray/black wall), not just dim-and-colourful (a
# shadowed orange line or red/green obstacle edge was bleeding into
# this range before since it only checked Value).
black_range = [
    [np.array([0, 0, 0]), np.array([180, 90, 125])]
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

magenta_range = [
    [np.array([150, 50, 120]), np.array([175, 255, 255])]
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

def navigate_wall(gyro_heading, desired_heading=0):
    """
    Blends two steering estimates into one value:
      1. Gyro term: proportional correction on heading error (gyro_heading vs desired_heading).
      2. Camera term: proportional correction on left/right wall pixel area difference (original logic).
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [30, 150].
    """
    # Refresh the side frames with the latest camera capture and re-run the
    # colour mask + contour detection so we know how much "wall" each side sees.
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)


    # Camera term (unchanged from original): more black pixels on one side
    # pushes steering away from that side, proportional to the area gap.
    cam_steer = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)


    # Gyro term: drives heading error toward zero. Falls back to straight if gyro unavailable.
    if gyro_heading is not None:
        heading_error = angle_error(gyro_heading, desired_heading)
        gyro_steer = DEFAULT_STEER_ANGLE - KP_GYRO * heading_error
    else:
        gyro_steer = DEFAULT_STEER_ANGLE


    # Weighted blend of the two independent steering estimates.
    steering_value = GYRO_WEIGHT * gyro_steer + CAM_WEIGHT * cam_steer
    steering_value = max(30, min(150, steering_value))  # clamp to servo range


    print(f"gyro heading: {gyro_heading:.0f}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, steer: {steering_value:.0f}")


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
left_frame = Frame(cap, 0, 80, 60, 200, colour_range=[black_range])
right_frame = Frame(cap, 240, 320, 60, 200, colour_range=[black_range])
bottom_frame = Frame(cap, 100, 220, 200, 240, colour_range=[blue_range, orange_range])
middle_frame = Frame(cap, 0, 320, 40, 220, colour_range=[red_obstacle_range, green_obstacle_range])
# Full frame width, not a narrow centre strip: during the avoidance turn the obstacle
# drifts sideways in-frame, and a narrow ROI was clipping/losing it well before the robot
# had actually passed it.
# Full frame: only watched while state is PARKING (see main loop), so it never competes
# with the wall/obstacle/turn ROIs during normal driving.
magenta_frame = Frame(cap, 0, 320, 0, 240, colour_range=[magenta_range])


print("ENTERING THE WHILE LOOP")


while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable
    if state == State.PARKING:
        magenta_frame.update(cap)
        magenta_contours = magenta_frame.find_contours()
        magenta_area, _ = magenta_frame.get_areas(magenta_contours)
        cv2.putText(cap, f"PARKING - magenta area: {magenta_area:.0f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # More black wall visible on one side -> steer away from that side (same
        # camera term as navigate_wall's cam_steer, just without the gyro blend).
        left_frame.update(cap)
        right_frame.update(cap)
        left_contours = left_frame.find_contours()
        right_contours = right_frame.find_contours()
        left_area, _ = left_frame.get_areas(left_contours)
        right_area, _ = right_frame.get_areas(right_contours)

        steering = DEFAULT_STEER_ANGLE + KP * (left_area - right_area)
        steering = int(max(30, min(150, steering)))
        speed = 60  # force the turn out of the parking spot, at the slowest speed used anywhere in this file, for precision in the tight parking space

        # Turn until the magenta marker clears the frame (area drops below
        # PARKING_MAGENTA_EXIT_AREA), gated by a minimum turn time so we don't exit
        # before the turn has actually started pulling the marker out of view, and
        # capped by a maximum turn time in case the marker never clears.
        parking_elapsed = time.time() - parking_turn_time
        if parking_elapsed > PARKING_MIN_TURN_TIME and (
            magenta_area < PARKING_MAGENTA_EXIT_AREA or parking_elapsed > PARKING_MAX_TURN_TIME
        ):
            if gyro is not None:
                desired_heading = gyro  # re-anchor heading hold to the actual post-turn heading, not the stale startup one
            state = State.WALL_FOLLOW
    else:
        steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
        speed =75

        # Update the middle frame and check for obstacles
        middle_frame.update(cap)
        red_contours, green_contours = middle_frame.find_contours()
        red_area, _ = middle_frame.get_areas(red_contours)
        green_area, _ = middle_frame.get_areas(green_contours)
    
        if red_area > 400 or green_area > 400:
            was_reversing = (state == State.REVERSING)  # captured before the state overwrite below
            state = State.AVOIDING_OBSTACLE  # block is in view; hold off on executing a queued turn until it's clear
            print(f"Red area: {red_area}, Green area: {green_area}")

            # Every contour across both colours, filtered down to real blobs (drop
            # single-pixel noise), ranked by how close its bottom edge is to the bottom of
            # the ROI (largest y = nearest the robot). The first is the closer contour, the
            # second (if any -- there's only ever 0 or 1 more) is the farther one.
            all_contours = (
                [(c, "RED") for c in red_contours if cv2.contourArea(c) > 5] +
                [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > 5]
            )
            all_contours.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

            closest_contour, obstacle_color = all_contours[0]
            obstacle_area = cv2.contourArea(closest_contour)

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
            speed = int(75 - proximity * (75 - 60))  # 75 far away, 60 once close -- tune on track

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
                    speed = 80
                else:
                    controller = "FWD"  # reverse window elapsed, state already AVOIDING_OBSTACLE above

     ### obstacle avoidance -- finding the points to plot and follow

            # On the obstacle's own row, look for the closest black pixel on the pass
            # side -- restricted to left_frame's/right_frame's own ROI columns, since
            # those are the actual wall-watching strips.
            full_hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
            full_black_mask = cv2.inRange(full_hsv, black_range[0][0], black_range[0][1])

            black_wall_x = None
            if 0 <= obstacle_y < full_black_mask.shape[0]:
                xs = np.nonzero(full_black_mask[obstacle_y])[0]
                if obstacle_color == "GREEN":
                    xs = xs[(xs >= left_frame.x1) & (xs < left_frame.x2)]
                    if xs.size > 0:
                        black_wall_x = int(xs.max())  # right-most black pixel in the left ROI
                else:
                    xs = xs[(xs >= right_frame.x1) & (xs < right_frame.x2)]
                    if xs.size > 0:
                        black_wall_x = int(xs.min())  # left-most black pixel in the right ROI

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
                cv2.putText(cap, f"Error: {cam_error} px", (target_x - 60, target_y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)  # Pink

                if state != State.REVERSING:
                    steering = int(max(30, min(150, DEFAULT_STEER_ANGLE + KP_OBSTACLE * cam_error)))
            elif state != State.REVERSING:
                # No wall point ever found for this obstacle -- fall back to a hard turn toward
                # this colour's correct pass side (right for RED, left for GREEN) instead of
                # always turning right, which would steer straight into a GREEN obstacle.
                steering = 150 if obstacle_color == "RED" else 30

            cv2.putText(cap, f"{obstacle_color} OBSTACLE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(cap, f"Speed: {speed}", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            last_target = None  # obstacle cleared; don't carry a stale target into the next one
            state = State.WALL_FOLLOW  # no block in view; may be upgraded to TURNING below
            steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
            speed = 75


        # Only look for a new turn-colour line if we're outside the "just turned" cooldown window.
        # Independent of `state`/obstacle-avoidance on purpose: an obstacle mid-turn-window should
        # still get avoided, and the cooldown must hold regardless of drive mode.
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
                turning = True
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


        if pending_turn and state not in (State.AVOIDING_OBSTACLE, State.REVERSING): # if a turn is pending and no block is currently being avoided, execute it
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


        # Once either colour has been crossed LINE_COUNT times, start the stop sequence.
        # Gated on `stopping`, not `state`: `state` is reassigned every frame by the
        # obstacle-detection branch above, so checking `state != State.STOPPED` here would
        # reset stop_time on every iteration and the 1s delay below would never elapse.
        if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
            if not stopping:
                stop_time = time.time()
            stopping = True

        if stopping:
            state = State.STOPPED  # override for display; overwritten again next frame, which is fine
            if time.time() - stop_time > 2:
                speed = 0000
                ser.write(f"90,0,STOP,0,stop\n".encode())  # send the fixed stop command
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


        if draw:
            cv2.imshow("Video Frame", cap)

    print(steering)
    print(f"STATE: {state.value}{' +TURNING' if turning else ''}")

    if abs(sent_steer - steering) >=3:
        print(steering)
        ser.write(f"{steering-22},{speed},{controller},{orange_count},open\n".encode())  # send steering+speed to the microcontroller each loop
        sent_steer = steering
        ser.flush()
    time.sleep(0.01)
    # print(f"sent value: heading: {steering} speed: {speed}")
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"90,0,STOP,0,stop\n".encode())
        ser.flush()
        break

ser.write(f"90,0,STOP,0,stop\n".encode())
ser.close()
       

cv2.destroyAllWindows()
