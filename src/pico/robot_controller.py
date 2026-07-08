import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

time.sleep(2)

print("READY")

while True:

    line = ser.readline().decode().strip()

    if line:

        print("RX FROM PICO:", line)

        # =========================
        # SEND REPLY BACK
        # =========================
        if line == "SENT OPEN BUTTON":
            

            servo_value = 0
            ser.write(f"{servo_value}\n".encode())
            print("somethin sent")
        
        elif line == "SENT OBSTACLE BUTTON":
            servo_value = 90
            ser.write(f"{servo_value}\n".encode())
            print("90")

        elif line == "SENT STOP BUTTON":
            servo_value = 180
            ser.write(f"{servo_value}\n".encode())
            print("180")

        #else:
            #break

