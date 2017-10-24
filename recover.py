#Method that will get current position written to robotPos.txt
#and return it to the start position.
#This is used when the robot has to be force stopped and needs to recover. 
#Only to be used in emergencies. 

from Movement import *
from Robot import *

try:
    f = open("/tmp/robotPos.txt", "r")
    data = f.read().split(",")
    data = [int(i) for i in data]

    m = Movement()
    m.current_pos = data[0:2]
    m.current_rot = data[2]
    m.reset_position()

    r = Robot()
    r.reset()
except:
    print "Error"
