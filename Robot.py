# Script to form as a wrapper class for the Robot
# Contains all functions for movement and the features
__author__ = 'Josh di Bona'

from Movement import *
from Camera import *
from Infrared import *
from LED import *
from Ultrasonic import *





class Robot(Singleton):

    movement = Movement()
    camera = Camera()
    ultrasonic = Ultrasonic()
    led = LED()
    ir = Infrared()



    def __init__(cls):
        print("Created Robot")

    def forward(cls, steps = 1):
        cls.movement.forward(steps)

    def reverse(cls, steps = 1):
        cls.movement.reverse(steps)

    def left(cls, steps = 1):
        cls.movement.left(steps)

    def right(cls, steps = 1):
        cls.movement.right(steps)

    def stop(cls):
        cls.movement.stop()

    def set_led_red(cls, state):
        cls.movement.set_led_red(state)

    def set_led_green(cls, state):
        cls.movement.set_led_green(state)

    def cleanup(cls):
        cls.movement.cleanup()

    def reset_position(cls):
        cls.movement.reset_position()

    def cont_forward(cls):
        cls.movement.cont_forward()

    def cont_reverse(cls):
        cls.movement.cont_reverse()

    def cont_left(cls):
        cls.movement.cont_left()

    def cont_right(cls):
        cls.movement.cont_right()

    def follow_path(cls, nodes_path):
        cls.movement.follow_path(nodes_path)

    def change_rot(cls, current_rot, goal_rot):
        cls.movement.change_rot(current_rot, goal_rot)

    def get_pos(cls):
        return cls.movement.get_pos()

    def get_rot(cls):
        return cls.movement.get_rot()

    def add_obstacles(cls, obstacles):
        cls.movement.add_obstacles(obstacles)

    def reset_obstacles(cls):
        cls.movement.reset_obstacles()

    def pathfind(cls, start_pos, goal_pos):
        return cls.movement.pathfind(start_pos, goal_pos)

    def set_start_pos(cls, x, y):
        cls.movement.set_start_pos(x, y)

    def set_start_rot(cls, rotation):
        cls.movement.set_start_rot(rotation)

    def get_start_pos(cls):
        return cls.movement.get_start_pos()

    def get_start_rot(cls):
        return cls.movement.get_start_rot()

    def print_grid(cls):
        cls.movement.print_grid()



























