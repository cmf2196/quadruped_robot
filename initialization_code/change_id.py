
'''
Author: Connor Finn
Date: June 27, 2020

This is a scrypt that will change a motor's id and set it to 120.
'''

from lx16a import *

LX16A.initialize("/dev/ttyUSB0")

servo1 = LX16A(1)

# Changes servo1's ID to 11
servo1.IDWrite(43)


