

'''
Author: Connor Finn
Date: August, 3rd, 2020
Description:
	This file will be an exploratory use of the camera. Eventually,
I would like to make this hold a camera class that we can import into 
our project.

https://elinux.org/RPi-Cam-Web-Interface


'''
from picamera import PiCamera
from config import camera_rotation

# initialize camera global (once and only once)
#camera = PiCamera()


class Camera():

	def __init__(self , camera):
		self.cam = camera
		self.cam.rotation = camera_rotation     # The camera is not oriented at 0 degrees

	def show(self):
		print('here')
		self.cam.start_preview()

	def end_show(self):
		self.cam.stop_preview()

	def take_picture(self):
		pass

	def take_video(self ):
		pass

if __name__ == "__main__":
#	c = Camera(camera)
#	c.show()
	from time import sleep
#	sleep(2)
#	c.end_show()
#	print('finished')
	camera = PiCamera()
	camera.start_preview()
	sleep(2)
	camera.stop_preview()
