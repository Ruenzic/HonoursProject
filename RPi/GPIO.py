# Template GPIO module for testing on windows. Before testing on Robot/Pi.

BOARD = 1
OUT = 1
IN = 1

class GPIO:

    def setmode(a):
       print(a)
    def setup(a, b):
       print(a)
    def output(a, b):
       print(a)
    def cleanup():
       print('a')
    def setmode(a):
       print(a)
    def setwarnings(flag):
       print('False')