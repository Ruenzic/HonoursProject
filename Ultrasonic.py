import RPi.GPIO as GPIO
import time

# Author: Jeremy Coupland
# Last updated: 2016/09/15

class Ultrasonic:

    def __init__(self):
        self.__TRIG = 38
        self.__ECHO = 36

    def __del__(self):
        GPIO.setwarnings(False)
        GPIO.cleanup()

    def get_distance(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(self.__TRIG,GPIO.OUT) # Set trigger as output
        GPIO.setup(self.__ECHO,GPIO.IN) # Set echo as input

        GPIO.output(self.__TRIG, False) # Make sure trigger isn't sending anything
        time.sleep(1) # Let the sensor settle

        distances = 0 # Distance initialized to 0
        for i in range (0,3): # Take average of 3 measurements
            GPIO.output(self.__TRIG, True)
            time.sleep(0.00001) # Pulse trigger for 10 microseconds
            GPIO.output(self.__TRIG, False)
            start = time.time()

            while GPIO.input(self.__ECHO)==0:
              start = time.time() # Start of the pulse

            while GPIO.input(self.__ECHO)==1:
              stop = time.time() # End of the pulse

            elapsed = stop-start  # Length of the pulse
            distance = (elapsed * 34300)/2 # Manipulating the Distance = Speed * Time equation

            distances = distances + distance # Keeping a running total of the measured distances

            time.sleep(0.1) # Brief wait between next measurement

        distances = distances/3 # Average distance using the three measurements

        return round(distances,2) # Distance in centimeters, rounded to 2 decimal places
