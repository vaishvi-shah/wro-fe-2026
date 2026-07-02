'''
1. make it run and stop when it sees black
2. if i see orange on floor, i turn right, blue turn left
'''

# 1 ----------------------------------------------------------------------------------------

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
from frames import Frame
import time
import serial
import bno055 

CALIBRATION_FILE = "src/sensors/bno055_calibration.json"
bno055.initialize()            # boot the IMU over I2C
sensor = bno055.sensor
bno055.load_calibration()      # apply saved accel/gyro/mag offsets if present

SHOW_VID = True                 # toggle live OpenCV preview window
DEFAULT_STEER_ANGLE = 90        # neutral/straight steering angle, sent as 100 + this
LINE_COUNT = 4000000                  # number of colour-line crossings before stopping

KP = 0.01       # camera proportional gain (wall pixel area difference)
KD = 0.001      # (unused currently, reserved for derivative term)
KP_GYRO = 0.5   # gyro proportional gain (heading error in degrees)
controller = 1

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
steering = 100 + DEFAULT_STEER_ANGLE
stop = False
speed = 1023

blue_count = 0      # number of blue line crossings seen
orange_count = 0    # number of orange line crossings seen
direction = ''      # locked turn direction once first colour is seen ("CWR" or "CCWL")

frame_count = 0      # frames seen since last FPS sample
fps = 0
frame_time = time.time()

turning = False           # true while inside the post-detection "turn window"
turning_time = time.time()  # timestamp of the last colour-line detection

# defining colour ranges (HSV) used to mask each region of interest
bottom_black = np.array([0, 0, 0])
high_black = np.array([180, 200, 60])
bottom_blue = np.array([100, 50, 60])
high_blue = np.array([140, 255, 255])
bottom_orange = np.array([10, 60, 100])
high_orange = np.array([25, 255, 255])

# Function to navigate straight along the wall based on the number of black pixels on either wall
# if more black on a wall, turn steering the other way proportional to difference of black pixels

desired_heading = 0

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
        signed_heading = heading_to_signed(gyro_heading)  # convert raw 0-359 reading to signed angle
        heading_error = signed_heading - desired_heading  # positive = drifted left, negative = drifted right
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
left_frame = Frame(cap, 0, 20, 120, 240,[bottom_black], [high_black])
right_frame = Frame(cap, 300, 320, 120, 240,[bottom_black], [high_black])
bottom_frame = Frame(cap, 100, 220, 200, 240,[bottom_blue], [high_blue], [bottom_orange], [high_orange])

print("ENTERING THE WHILE LOOP")

while True:
    cap = picam2.capture_array("main")     # latest camera frame
    gyro = bno055.get_heading()            # latest raw heading (0-359 deg), or None if unavailable                   # target heading: straight ahead
    steering = 100 + navigate_wall(gyro, desired_heading)  # blended gyro+camera steering, offset for serial protocol
    speed = 1023

    # Only look for a new turn-colour line if we're outside the "just turned" cooldown window.
    if time.time() - turning_time > 2:
        bottom_frame.update(cap)
        blue_contours, orange_contours = bottom_frame.find_contours(colour=(255,255,0), colour2=(0,127,255))
        bottom_area, bottom_colour = bottom_frame.get_areas(blue_contours, orange_contours) # if bottom_colour = 1 = blue if bottom_colour = 2 = orange
        # print("GOT THE AREAS")

        # A large enough patch of blue/orange counts as a line crossing.
        if bottom_area != 0 and bottom_area > 1600:
            # print(time.time(), "--Detected turn color--")
            turning_time = time.time()  # restart the cooldown window
            turning = True # This is only used for debug purposes to indicate end if turn
            if bottom_colour == 1:
                blue_count += 1
                desired_heading -= 90
                print(f"BLUE: {blue_count}")
                if not direction:
                    direction = "CWR"  # lock turn direction on first colour seen
            elif bottom_colour == 2:
                orange_count += 1
                desired_heading += 90
                print(f"ORANGE: {orange_count}")
                if not direction:
                    direction = "CCWL"


        


    else:
        # Debug: mark end of turn window
        if turning:
            turning = False

    # Once either colour has been crossed LINE_COUNT times, start the stop sequence.
    if orange_count >= LINE_COUNT or blue_count >= LINE_COUNT:
        if not stop:
            stop_time = time.time()
        stop = True
    
    if stop:
        if(time.time() - stop_time > 1):  # short grace period before actually stopping
            print("Stopping")
            speed = 0000
            ser.write(f"19010220\n".encode())  # send the fixed stop command
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


    ser.write(f"{steering:.0f}{speed}{controller}\n".encode())  # send steering+speed to the microcontroller each loop
    ser.flush()
    time.sleep(0.01)
    # print(f"sent value: heading: {steering} speed: {speed}")
    if cv2.waitKey(1) & 0xFF == ord('q'):  # manual quit key also sends the stop command
        ser.write(f"1901023{controller}\n".encode())
        ser.flush()
        break
ser.write(f"1901023{controller}\n".encode())
ser.flush()
ser.close()
        

cv2.destroyAllWindows()


