# 1 ----------------------------------------------------------------------------------------

# WORKING FINAL COPY -----

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
Two controllers “compete” and are averaged.


Core idea:


“Both sensors directly output steering, then we mix them.”


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
controller = "FWD"
sent_steer = 0
SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 100        # neutral/straight steering angle
LINE_COUNT = 12                # number of colour-line crossings before stopping
SAFE_TURN_AREA = 3000           # max black area on the sid of a turn before we can safely execute the turn
KP = 0.05       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = DEFAULT_STEER_ANGLE
stop = False
speed = 80
pending_turn = False  # true if a turn is pending (colour line seen, but not yet executed)


blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")


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
    Final steering = 70% gyro term + 30% camera term, clamped to servo range [45, 135] (90 +/- 45).
    """
    # Refresh the side frames with the latest camera capture and re-run the
    # colour mask + contour detection so we know how much "wall" each side sees.
    left_frame.update(cap)
    right_frame.update(cap)


    left_contours = left_frame.find_contours()
    right_contours = right_frame.find_contours()


    left_area, _ = left_frame.get_areas(left_contours)
    right_area, _ = right_frame.get_areas(right_contours)

    # Blue seen first (CCL) -> bias steering further left by padding the
    # right-side black pixel count.
    if direction == "CCL":
        right_area += 1000

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
    steering_value = max(45, min(135, steering_value))  # clamp to servo range


    print(f"gyro heading: {gyro_heading:.0f}, gyro steer: {gyro_steer:.0f}, cam steer: {cam_steer:.0f}, steer: {steering_value:.0f}")

    if SHOW_VID:
        cv2.putText(cap,
                    f"steer={GYRO_WEIGHT}*{gyro_steer:.1f}+{CAM_WEIGHT}*{cam_steer:.1f}={steering_value:.1f}",
                    (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)


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

print("ENTERING THE WHILE LOOP")


while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable
    steering = navigate_wall(gyro, desired_heading)  # blended gyro+camera steering


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




    # Once either colour has been crossed LINE_COUNT times, start the stop sequence.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True
   
    if stop:
        if time.time() - stop_time > 2:
            speed = 0
            ser.write(f"{DEFAULT_STEER_ANGLE},0,STOP,0,test\n".encode())  # send the fixed stop command
            ser.flush()
            break




    if (SHOW_VID):
        # ROI borders, drawn last (not in Frame.update()) so a border baked
        # early doesn't leak into another Frame's crop later -- see Frame.draw_roi().
        left_frame.draw_roi(cap)
        right_frame.draw_roi(cap)
        bottom_frame.draw_roi(cap)

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
        ser.write(f"{steering-5},{speed},{controller},{LINE_COUNT},'test'\n".encode())  # send steering+speed to the microcontroller each loop
        sent_steer = steering
        ser.flush()
    time.sleep(0.01)
    # print(f"sent value: heading: {steering} speed: {speed}")
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"{DEFAULT_STEER_ANGLE},0,STOP,0,test\n".encode())
        ser.flush()
        break

ser.write(f"{DEFAULT_STEER_ANGLE},0,STOP,0,test\n".encode())
ser.close()
       

cv2.destroyAllWindows()