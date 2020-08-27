'''
Author: Connor Finn
Date: August 26, 2020

Description:  
     This class will be the robot class, it should have methods such as 

'''

from lx16a import *


class Robot():


	def __init__(self):
		from config import motor_ids
		servos = [LX16A(motor) for motor in motor_ids]


	def turn_motors_on(self):
		for servo in servos:
    		servo.loadOrUnloadWrite(1)

    def stand(self , time):
    	from config import stand_pos

    	vals = [(pos , time) for pos in stand_pos]
