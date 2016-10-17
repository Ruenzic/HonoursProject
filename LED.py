import RPi.GPIO as GPIO
import time

#Author: Jeremy Coupland
#Last updated: 2016/09/15

class LED:

    def __init__(self):
        self.__RedPin = 33
        self.__BluePin = 35
        self.__YellowPin = 37

        

    def __del__(self):
        self.__colourOff(self.__RedPin)
        self.__colourOff(self.__BluePin)
        self.__colourOff(self.__YellowPin)
        GPIO.setwarnings(False)
        GPIO.cleanup()



    def __colourOn(self, pinNumber):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        colourPin = pinNumber
        GPIO.setup(colourPin, GPIO.OUT)

        GPIO.output(colourPin, True) #Turns the LED on
        time.sleep(0.11)


    def __colourOff(self, pinNumber):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        colourPin = pinNumber
        GPIO.setup(colourPin, GPIO.OUT)

        GPIO.output(colourPin, False) #Turns the LED off
        time.sleep(0.1)

    def __colourFlash(self, pinNumber):
	switchBack = False	

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        colourPin = pinNumber
        GPIO.setup(pinNumber, GPIO.OUT)

	if (GPIO.input(pinNumber) == 1):  #If the light is already on
        	switchBack = True


	GPIO.output(pinNumber, False) #Turns the LED off 
	time.sleep(0.2)
        GPIO.output(pinNumber, True) #Turns the LED on
        time.sleep(0.3)
        GPIO.output(pinNumber, False) 
	
	if switchBack:
		time.sleep(0.2)
		GPIO.output(pinNumber, True)


    def red_on(self):
        self.__colourOn(self.__RedPin)



    def red_off(self):
       self.__colourOff(self.__RedPin)
        


    def red_flash(self):
        self.__colourFlash(self.__RedPin)


    def blue_on(self):
        self.__colourOn(self.__BluePin)



    def blue_off(self):
        self.__colourOff(self.__BluePin)
        


    def blue_flash(self):
        self.__colourFlash(self.__BluePin)


    def yellow_on(self):
        self.__colourOn(self.__YellowPin)



    def yellow_off(self):
        self.__colourOff(self.__YellowPin)
        


    def yellow_flash(self):
        self.__colourFlash(self.__YellowPin)





    




