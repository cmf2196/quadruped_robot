'''
Author: Connor Finn
Date:  August 31
Description:
	This class will initialize the imu sensor and have methods for reading its data
	The BNO05vsensor, https://www.adafruit.com/product/2472 , can read the following data, 
		- Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
		- Absolute Orientation (Quatenrion, 100Hz) Four point quaternion output for more accurate data manipulation
		- Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
		- Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
		- Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
		- Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
		- Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
		- Temperature (1Hz) Ambient temperature in degrees celsius
'''

from Adafruit_BNO055 import BNO055
import time

class Imu():


	def calibrate(self):
		pass



	def read_orientation(self):
		pass


	def read_accel(self):
		pass
