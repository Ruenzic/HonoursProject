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

    def forward(cls, steps):
        cls.movement.forward(steps)

    def reverse(cls, steps):
        cls.movement.reverse(steps)













