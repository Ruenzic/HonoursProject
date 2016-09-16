# Script to form as a wrapper class for the Robot
# Contains all functions for movement and the features
__author__ = 'Josh di Bona'

from Movement import *
from Camera import *
from Infrared import *
from LED import *
from Ultrasonic import *


class Robot(Singleton):

    def __init__(self):
        self.__movement = Movement()
        self.camera = Camera()
        self.ultrasonic = Ultrasonic()
        self.led = LED()
        self.ir = Infrared()
        print("Created Robot")

    #Movement Functions

    def forward(self, steps = 1):
        self.__movement.forward(steps)

    def reverse(self, steps = 1):
        self.__movement.reverse(steps)

    def left(self, steps = 1):
        self.__movement.left(steps)

    def right(self, steps = 1):
        self.__movement.right(steps)

    def stop(self):
        self.__movement.stop()

    def cleanup(self):
        self.__movement.cleanup()

    def reset_position(self):
        self.__movement.reset_position()

    def cont_forward(self):
        self.__movement.cont_forward()

    def cont_reverse(self):
        self.__movement.cont_reverse()

    def cont_left(self):
        self.__movement.cont_left()

    def cont_right(self):
        self.__movement.cont_right()

    def follow_path(self, nodes_path):
        self.__movement.follow_path(nodes_path)

    def change_rot(self, current_rot, goal_rot):
        self.__movement.change_rot(current_rot, goal_rot)

    def get_pos(self):
        return self.__movement.get_pos()

    def get_rot(self):
        return self.__movement.get_rot()

    def add_obstacles(self, obstacles):
        self.__movement.add_obstacles(obstacles)

    def reset_obstacles(self):
        self.__movement.reset_obstacles()

    def pathfind(self, start_pos, goal_pos):
        return self.__movement.pathfind(start_pos, goal_pos)

    def set_start_pos(self, x, y):
        self.__movement.set_start_pos(x, y)

    def set_start_rot(self, rotation):
        self.__movement.set_start_rot(rotation)

    def get_start_pos(self):
        return self.__movement.get_start_pos()

    def get_start_rot(self):
        return self.__movement.get_start_rot()

    def print_grid(self):
        self.__movement.print_grid()

    def reset(self):
        self.led.blue_off()
        self.led.red_off()
        self.led.yellow_off()

        self.__movement.reset_position()
        self.__movement.set_led_green(0)
        self.__movement.set_led_red(0)
        self.__movement.cleanup()























