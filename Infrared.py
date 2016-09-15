import RPi.GPIO as GPIO
import time

#Author: Jeremy Coupland
#Last updated: 2016/09/15

class Infrared():

    def __init__(self):
        self.__IRPin = 40


    def __del__(self):
        GPIO.setwarnings(False)
        GPIO.cleanup()
    
        

    def detect_obstacle(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.__IRPin, GPIO.IN)

        if (GPIO.input(self.__IRPin) == 1):    #If no obstacle is being detected
            return False

        elif (GPIO.input(self.__IRPin) == 0):  #If an obstacle is detected
            return True     



    def wait_for_obstacle(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.__IRPin, GPIO.IN)

        timeElapsed = 0     #For keeping track of time, to prevent infinite loop
        found = False       #Default is no object detected

        while (True):
            if (GPIO.input(self.__IRPin) == 0):    #If an object is detected
                found = True
                break
            
            time.sleep(0.05)    #Otherwise, wait for a short period, and check again
            timeElapsed = timeElapsed + 0.05

            if timeElapsed > 10:    #Stop looping after 10 seconds
                break
        

        return found    #Will return True or False
