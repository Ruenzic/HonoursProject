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
