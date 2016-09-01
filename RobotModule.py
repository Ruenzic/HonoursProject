# RobotModule.py Library

import RPi.GPIO as GPIO
import time


class Robot:

    RIGHT_PWM_PIN = 10
    RIGHT_1_PIN = 10
    RIGHT_2_PIN = 25
    LEFT_PWM_PIN = 17
    LEFT_1_PIN = 17
    LEFT_2_PIN = 4
    SW1_PIN = 11
    SW2_PIN = 9
    LED1_PIN = 8
    LED2_PIN = 7
    OC1_PIN = 22
    OC2_PIN = 27
    OC2_PIN_R1 = 21
    OC2_PIN_R2 = 27
    TRIGGER_PIN = 18
    ECHO_PIN = 23
    left_pwm = 0
    right_pwm = 0
    pwm_scale = 0

    old_left_dir = -1
    old_right_dir = -1

    move_delay = 0.4 #See what happens when you change delay
    voltage = 5
    left_voltage_scale = 1
    right_voltage_scale = 1

    step_time = 0.5

    turn_time = 0.4

    start_pos = [0,0]
    start_rot = 0
    current_pos = [0,0]
    current_rot = 0

    def __init__(self,revision=2):

        self.pwm_scale = 1

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, 500)
        self.left_pwm.start(0)
        GPIO.setup(self.LEFT_1_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_2_PIN, GPIO.OUT)

        GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, 500)
        self.right_pwm.start(0)
        GPIO.setup(self.RIGHT_1_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_2_PIN, GPIO.OUT)

        GPIO.setup(self.LED1_PIN, GPIO.OUT)
        GPIO.setup(self.LED2_PIN, GPIO.OUT)

        GPIO.setup(self.OC1_PIN, GPIO.OUT)
        if revision == 1:
            self.OC2_PIN = self.OC2_PIN_R1
        else:
            self.OC2_PIN = self.OC2_PIN_R2

        GPIO.setup(self.OC2_PIN_R2, GPIO.OUT)

        GPIO.setup(self.SW1_PIN, GPIO.IN)
        GPIO.setup(self.SW2_PIN, GPIO.IN)
        GPIO.setup(self.TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

    def set_motors(self, left_pwm, left_dir, right_pwm, right_dir):
        self.set_driver_pins(left_pwm, left_dir, right_pwm, right_dir)
        self.old_left_dir = left_dir
        self.old_right_dir = right_dir

    def set_driver_pins(self, left_pwm, left_dir, right_pwm, right_dir):
        self.left_pwm.ChangeDutyCycle(left_pwm * 100 * self.pwm_scale)
        GPIO.output(self.LEFT_1_PIN, left_dir)
        GPIO.output(self.LEFT_2_PIN, not left_dir)
        self.right_pwm.ChangeDutyCycle(right_pwm * 100 * self.pwm_scale)
        GPIO.output(self.RIGHT_1_PIN, right_dir)
        GPIO.output(self.RIGHT_2_PIN, not right_dir)

    def forward(self, steps = 1): # 1 step by default
        for num in range(0,steps):
            self.set_motors(self.left_voltage_scale, 1, self.right_voltage_scale, 1)
            time.sleep(self.step_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.manage_pos(steps)

    def stop(self):
        self.set_motors(0, 0, 0, 0)

    def reverse(self, steps = 1): # 1 step by default
        for num in range(0,steps):
            self.set_motors(self.left_voltage_scale, 0, self.right_voltage_scale, 0)
            time.sleep(self.step_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.manage_pos(steps * -1) # Make the steps negative so that the manage_pos func will move robot in correct dir based on its rot

    def left(self, steps=1): # 45 degrees by default
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.set_motors(self.left_voltage_scale, 0, self.right_voltage_scale, 1)
            time.sleep(self.turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.current_rot -= steps
        self.manage_rot()

    def right(self, steps=1): # 45 degrees by default
        if (steps >= 8):
            steps -= 8 # Remove 360 degree turns
        for num in range(0,steps):
            self.set_motors(self.left_voltage_scale, 1, self.right_voltage_scale, 0)
            time.sleep(self.turn_time)
            self.stop()    # Delay between each movement
            time.sleep(self.move_delay)
        self.current_rot += steps
        self.manage_rot()

    def set_led1(self, state):
        GPIO.output(self.LED1_PIN, state)

    def set_led2(self, state):
        GPIO.output(self.LED2_PIN, state)

    def cleanup(self):
        GPIO.cleanup()

    def manage_rot(self): # Manage the current rotation by removing 360 degrees when needed.
        if (self.current_rot >= 8):
            self.current_rot -= 8
        elif (self.current_rot < 0):
            self.current_rot += 8

    def manage_pos(self, steps):

        self.manage_rot() # Make sure rotation is in positive form for ease

        if (self.current_rot == 0): # Increase the y value by the number of steps
            self.current_pos[1] += steps
        elif (self.current_rot == 1): # Increase x and y
            self.current_pos[0] += steps
            self.current_pos[1] += steps
        elif (self.current_rot == 2): # Increase x
            self.current_pos[0] += steps
        elif (self.current_rot == 3): # Increase x, Decrease y
            self.current_pos[0] += steps
            self.current_pos[1] -= steps
        elif (self.current_rot == 4): # Decrease y
            self.current_pos[1] -= steps
        elif (self.current_rot == 5): # Decrease x and y
            self.current_pos[0] -= steps
            self.current_pos[1] -= steps
        elif (self.current_rot == 6): # Decrease x
            self.current_pos[0] -= steps
        elif (self.current_rot == 7): # Decrease x, Increase y
            self.current_pos[0] -= steps
            self.current_pos[1] += steps

    def reset(self):
        # Move the robot back to the start position
        # Move on the y axis
        if (self.current_pos[1] > 0):
            if (self.current_rot != 4):
                # rotate towards the start
                if (self.current_rot >= 0 and self.current_rot < 4):
                    self.right(4 - self.current_rot)
                elif (self.current_rot > 4):
                    self.left(self.current_rot - 4)
            self.forward(self.current_pos[1])

        elif (self.current_pos[1] < 0):
            if (self.current_rot != 0):
                # rotate towards the start
                if (self.current_rot >= 0 and self.current_rot < 4):
                    self.left(self.current_rot)
                elif (self.current_rot > 4):
                    self.right(8 - self.current_rot)
            self.forward(self.current_pos[1] * -1) # Change negative to positive

        # Move on the x axis
        if (self.current_pos[0] > 0):
            if (self.current_rot != 6):
                # rotate towards the start
                if (self.current_rot >= 2 and self.current_rot < 6):
                    self.right(6 - self.current_rot)
                elif (self.current_rot < 2):
                    self.left(2 + self.current_rot)
                elif (self.current_rot > 6):
                    self.left(7 - self.current_rot)
            self.forward(self.current_pos[0])

        elif (self.current_pos[0] < 0):
            if (self.current_rot != 2):
                # rotate towards the start
                if (self.current_rot >= 2 and self.current_rot < 6):
                    self.left(self.current_rot - 2)
                elif (self.current_rot < 2):
                    self.right(2 - self.current_rot)
                elif (self.current_rot > 6):
                    self.right(8 - self.current_rot + 1)
            self.forward(self.current_pos[0] * -1) # Change the negative to positive


        # Set the rotation of the robot back to the original direction
        # Check to see if rotation is bigger or smaller than 4, then rotate the shorter direction
        if (self.current_rot != 0):
            # rotate towards the start
            if (self.current_rot >= 0 and self.current_rot < 4):
                self.left(self.current_rot)
            elif (self.current_rot > 4):
                self.right(8 - self.current_rot)

























