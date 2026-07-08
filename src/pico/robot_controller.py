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
        if line == "Move Forward":
            servo_value = 0
            ser.write(f"0,1,CW,1,VAISHVI\n".encode())
            print(f"servo angle {servo_value}")
        
        elif line == "Move Backward":
            servo_value = 90
            ser.write(f"90,100,CW,100,JAITRA\n".encode())
            print(f"servo angle {servo_value}")

        elif line == "Stop":

            print("\n--- MOTOR COMMAND INPUT ---")

            line_count = 9 #input("Line color: ")

            direction = 'CW' #input("Direction (CW/CCW/STOP): ")

            speed = 1024 #input("Speed (0-100): ")

            angle = 110 #input("Servo angle (0-180): ")

            state = 'Avani'


            # Create one line of data
            command = f"{angle},{speed},{direction},{line_count},{state}"


            # Send command to Pico
            ser.write((command + "\n").encode())


            print("SENT TO PICO:", command)

        #else:
            #break

