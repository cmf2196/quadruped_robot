''' Author: Connor Finn Date: July 12, 2020 Description:
	This file has a function that will make the robot stand up 
	to a starting position.
'''

from config import motor_ids , stand_pos
from lx16a import *

# initialize the servo
LX16A.initialize("/dev/ttyUSB0")
servos = []
count = 0

'''
TO DO: Change from for loops to a map function
'''
for motor in motor_ids:
	print(motor)
	servos += [LX16A(motor)]
	count +=1



pre_loc_pos = [120, 235 , 70 , 120  , 5 ,170] *2

locs = []
time = 2000
for pos in pre_loc_pos:
	locs +=[(pos , time)]

#LX16A.moveTimeWriteList(servos , locs)

locs2 = []
time = 3000
for pos in stand_pos:
        locs2 +=[(pos , time)]

LX16A.moveTimeWriteList(servos , locs2)


