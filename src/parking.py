import cv2
import numpy as np
from picamera2 import Picamera2
from lib.frames import Frame
import time
import serial
import lib.bno055 as bno055

# HSV ranges
magenta_range = [[np.array([150, 50, 60]), np.array([175, 255, 255])]]
black_range   = [[np.array([0, 0, 0]),     np.array([180, 255, 60])]]  # tune V ceiling

STEER_OFFSET = 22   # protocol subtracts this before sending
DRIVE_SPEED  = 70
KP_GYRO      = 0.5  # gyro proportional gain (heading error in degrees -> steer degrees)
orange_count = 0    # unused here, but the pico protocol expects this field

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # serial link to the steering/speed microcontroller
time.sleep(2)  # let the serial connection settle before writing
sent_steer = None
sent_controller = None

print("-- INITIALIZING GYRO --")
bno055.initialize()
bno055.load_calibration()

cv2.startWindowThread()

print("-- INITIALIZING CAMERA --")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()
cap = picam2.capture_array("main")

# ---- ROIs (x1, x2, y1, y2) -- adjust to your mounting ----
pink_frame  = Frame(cap, 0,   320, 0,  240, colour_range=[magenta_range])  # whole screen -- magenta
black_left  = Frame(cap, 0,   70,  40, 200, colour_range=[black_range])    # left black wall
black_right = Frame(cap, 250, 320, 40, 200, colour_range=[black_range])    # right black wall


def areas(frame, img):
    frame.update(img)
    a, _ = frame.get_areas(frame.find_contours())
    return a


def angle_error(current, target):
    """Signed shortest angular distance from current to target, in degrees."""
    return (current - target + 180) % 360 - 180


def send(steer_val, speed_val, tag):
    global sent_steer, sent_controller
    if sent_steer is None or abs(sent_steer - steer_val) >= 3 or sent_controller != controller:
        ser.write(f"{steer_val - STEER_OFFSET},{speed_val},{controller},{orange_count},{tag}\n".encode())
        ser.flush()
        sent_steer = steer_val
        sent_controller = controller


# ---- one-time steer decision ----a
left_a  = areas(black_left,  cap)
right_a = areas(black_right, cap)
steer = 30 if right_a > left_a else 150
ext_direction = "left" if right_a > left_a else "right"

desired_heading = bno055.get_heading()  # heading to return to once the pink target clears the frame

SIDE_FRACTION = 0.75  # reverse once pink is entirely within this fraction of the frame, on one side
left_thresh_x  = cap.shape[1] * (1 - SIDE_FRACTION)  # 80  for a 320-wide frame
right_thresh_x = cap.shape[1] * SIDE_FRACTION        # 240 for a 320-wide frame

controller = "FWD"
bwd_until = 0  # timestamp BWD ends -- only meaningful while controller == "BWD"
reversed_once = False  # once the reverse trigger fires, stop checking for it
realigning = False  # once pink clears the frame post-reverse, steer back to desired_heading

print("ENTERING THE WHILE LOOP")

try:
    while True:
        cap = picam2.capture_array("main")

        pink_frame.update(cap)
        pink_contours = pink_frame.find_contours()
        pink_a, _ = pink_frame.get_areas(pink_contours)

        left_a  = areas(black_left,  cap)
        right_a = areas(black_right, cap)

        # drop speck-sized noise contours so a stray pixel doesn't count as "still there"
        real_pink   = [c for c in pink_contours if cv2.contourArea(c) > 5]
        pink_left   = any(c[:, :, 0].min() < left_thresh_x for c in real_pink)
        pink_right  = any(c[:, :, 0].max() >= right_thresh_x for c in real_pink)

        if not reversed_once:
            should_reverse = (ext_direction == "right" and not pink_right) or \
                             (ext_direction == "left" and not pink_left)
            if should_reverse:
                controller = "BWD"
                bwd_until = time.time() + 0.5
                reversed_once = True

        if controller == "BWD":
            if time.time() >= bwd_until:
                controller = "FWD"
                steer = 30 if ext_direction == "left" else 150
            else:
                steer = 90

        # once the pink target has fully left the frame post-reverse, stop chasing it
        # and steer back toward the heading we had before the exit turn
        if reversed_once and controller == "FWD" and not real_pink:
            realigning = True

        gyro_heading = None
        if realigning:
            gyro_heading = bno055.get_heading()
            if gyro_heading is not None and desired_heading is not None:
                heading_error = angle_error(gyro_heading, desired_heading)
                steer = int(max(30, min(150, 90 - KP_GYRO * heading_error)))
            else:
                steer = 90

        send(steer, DRIVE_SPEED, controller.lower())

        for txt, y in [(f"pink : {pink_a:.0f}",  20),
                       (f"left : {left_a:.0f}",  40),
                       (f"right: {right_a:.0f}", 60),
                       (f"steer: {steer}",       80),
                       (f"exit : {ext_direction}", 100),
                       (f"controller: {controller}", 120),
                       (f"pink L/R: {pink_left}/{pink_right}", 140),
                       (f"realign: {realigning} gyro: {gyro_heading}", 160)]:
            cv2.putText(cap, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        pink_frame.draw_roi(cap)
        black_left.draw_roi(cap)
        black_right.draw_roi(cap)

        cv2.imshow("Video Frame", cap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    ser.write(f"90,0,STOP,0,stop\n".encode())  # send the fixed stop command
    ser.flush()
    ser.close()
    cv2.destroyAllWindows()
