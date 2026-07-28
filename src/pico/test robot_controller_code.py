import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

time.sleep(2)

print("READY")

open_commands = [
    "0,1024,FWD,9,open",
    "45,800,FWD,10,open",
    "90,600,FWD,12,open",
    "135,400,FWD,15,open",
    "180,0,STOP,0,open"
]

obstacle_commands = [
    "180,0,STOP,0,obstacle",
    "135,400,FWD,15,obstacle",
    "90,600,FWD,12,obstacle",
    "45,800,FWD,10,obstacle",
    "0,1024,FWD,9,obstacle"
]

stop_command = "90,0,STOP,0,stop"

while True:

    line = ser.readline().decode().strip()


    if line:

        print("RX FROM PICO:", line)

        if line == "Open":

         print("\n--- AUTOMATIC COMMAND MODE ---")

         for command in open_commands:
            print("Sending:", command)
            ser.write((command + "\n").encode())
         
            time.sleep(1)

        


        elif line == "Obstacle":

            print("\n--- AUTOMATIC COMMAND MODE ---")

            for command in obstacle_commands:
                print("Sending:", command)
                ser.write((command + "\n").encode())
         
                time.sleep(1)

        

        elif line == "Stop":

            print("\n--- AUTOMATIC COMMAND MODE ---")

           
            print("Sending:", stop_command)
            ser.write((stop_command + "\n").encode())

            time.sleep(1)
         

        



      