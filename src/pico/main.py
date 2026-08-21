from machine import Pin, SPI, PWM
import framebuf
import time
import sys
import uselect
import utime

# 1. Setup Pins
# ENA controls speed using Pulse Width Modulation (PWM)
ena = machine.PWM(machine.Pin(4))
ena.freq(1000) # Set PWM frequency to 1kHz

# IN1 and IN2 control the motor direction
in1 = machine.Pin(6, machine.Pin.OUT)
in2 = machine.Pin(7, machine.Pin.OUT)

# 
BL = 13
DC = 8
RST = 12
MOSI = 11
SCK = 10
CS = 9

class LCD_1inch44(framebuf.FrameBuffer):

    def __init__(self):

        self.width = 128
        self.height = 128

        self.cs = Pin(CS, Pin.OUT)
        self.rst = Pin(RST, Pin.OUT)

        self.spi = SPI(
            
            1,
            baudrate=40000000,
            polarity=0,
            phase=0,
            sck=Pin(SCK),
            mosi=Pin(MOSI),
            miso=None
        )

        self.dc = Pin(DC, Pin.OUT)

        self.buffer = bytearray(self.width * self.height * 2)

        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)

        self.init_display()
        
        self.black = 0x0000
        self.white = 0xFFFF
        self.red   = 0xF800
        self.blue  = 0x07E0
        self.green = 0x001F
        

    def write_cmd(self, cmd):
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, data):
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([data]))
        self.cs(1)

    def init_display(self):

        self.rst(1)
        time.sleep_ms(50)
        self.rst(0)
        time.sleep_ms(50)
        self.rst(1)
        time.sleep_ms(50)

        self.write_cmd(0x11)
        time.sleep_ms(120)

        self.write_cmd(0x36)
        self.write_data(0x70)

        self.write_cmd(0x3A)
        self.write_data(0x05)

        self.write_cmd(0x29)

    def show(self):

        self.write_cmd(0x2A)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x7F)

        self.write_cmd(0x2B)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x7F)

        self.write_cmd(0x2C)

        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)


# Backlight
pwm = PWM(Pin(BL))
pwm.freq(1000)
pwm.duty_u16(65535)

LCD = LCD_1inch44()
LCD.init_display()
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

# -------------------------
# BUTTONS
# -------------------------

btn_stop = Pin(2, Pin.IN, Pin.PULL_UP)
btn_obstacle = Pin(3, Pin.IN, Pin.PULL_UP)
btn_open = Pin(17, Pin.IN, Pin.PULL_UP)
btn_meme = Pin(15, Pin.IN, Pin.PULL_UP)

# Servo initialization
servo = PWM(Pin(1))
servo.freq(50)

def set_angle(angle):
    min_us = 500
    max_us = 2500

    pulse_us = min_us + (angle / 180) * (max_us - min_us)

    duty = int(pulse_us / 20000 * 65535)
    servo.duty_u16(duty)

def show_message(msg, color):

    LCD.fill(color)

    LCD.text(msg, 10, 60, LCD.white)

    LCD.show()

def motor_forward(speed):
    """Moves the motor forward at a specific speed (0 to 100)."""
    in1.value(1)
    in2.value(0)
    # MicroPython PWM duty cycle goes from 0 to 65535
    duty_cycle = int((speed / 100) * 65535)
    ena.duty_u16(duty_cycle)
    print(f"Moving Forward at {speed}% speed")

def motor_backward(speed):
    """Moves the motor backward at a specific speed (0 to 100)."""
    in1.value(0)
    in2.value(1)
    duty_cycle = int((speed / 100) * 65535)
    ena.duty_u16(duty_cycle)
    print(f"Moving Backward at {speed}% speed")

def motor_stop():
    """Turns the motor off."""
    in1.value(0)
    in2.value(0)
    ena.duty_u16(0)
    print("Motor Stopped")
    

def handle_motor_commands(direction, speed_str, angle_str, line, state):
    try:
        speed = int(speed_str)
        angle = int(angle_str)
        
        set_angle(angle)
        
        if direction == "FWD":
            motor_forward(speed)
        elif direction == "BWD":
            motor_backward(speed)
        elif direction == "STOP":
            motor_stop()
    except ValueError:
        print("Data conversion error")

def clear_screen():
    LCD.fill(LCD.black)
    LCD.show()
    

def btn_open_close(ser_value, speed_value, direc, line_col, sta ):
    clear_screen()
    
    LCD.text("SerV: " + ser_value, 10, 10, LCD.white)
    LCD.text("SpV: " + speed_value, 10, 30, LCD.white)
    LCD.text("Dir:" + direc, 10, 50, LCD.white)
    LCD.text("LC:" + line_col, 10, 70, LCD.white)
    LCD.text("St:" + sta, 10, 90, LCD.white)
                    
    LCD.show()
    time.sleep(0.3)

while True:
    
    show_message("Welcome!!!", LCD.green)


    # -------------------------
    # OPEN BUTTON
    # -------------------------
    if btn_open.value() == 0:     
        show_message("Open", LCD.green)
        print("Open")        
        time.sleep(0.3)
            
    # -------------------------
    # OBSTACLE BUTTON
    # -------------------------
    elif btn_obstacle.value() == 0:
        
        show_message("Obstacle", LCD.green)
        print("Obstacle")
        time.sleep(1)
        

    # -------------------------
    # STOP BUTTON
    # -------------------------
    elif btn_stop.value() == 0:
#         motor_stop()
 
        show_message("STOP", LCD.red)
        #LCD.Paint_DrawLine(0, 10, 127, 10, LCD.black, 2, 0)
        print("Stop")
        time.sleep(0.3)
            
    # -------------------------
    # Team Name BUTTON
    # -------------------------
    elif btn_meme.value() == 0:
# THis is the message being sent from the Pico to Raspberry Pi 
        print("SENT Team Name")

        show_message("SENT Team Name", LCD.black)

        time.sleep(0.3)
    
    
    
    
    # =====================================
    # RECEIVE FROM PI (NON-BLOCKING)
    # =====================================
    
    poll = uselect.poll()
    poll.register(sys.stdin, uselect.POLLIN)
    
    
    
    if poll.poll(0):
        line = sys.stdin.readline().strip()
            
        if "," in line:

            try:
                servo_value, speed_val, direction, line_color, state = line.split(",")
                btn_open_close(servo_value, speed_val, direction, line_color, state)
                
                # This calls the function we defined at the top!
                handle_motor_commands(direction, int(speed_val), int(servo_value), int(line_color), state)
                    
            except ValueError:
                print("Error splitting data:", line)   


