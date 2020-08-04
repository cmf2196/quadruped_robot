'''
Author: Connor Finn
Date: August, 3rd, 2020
Description:
	This file will be an exploratory use of the camera. Eventually,
I would like to make this hold a camera class that we can import into 
our project.
'''
from picamera import PiCamera
from config import camera_rotation

# initialize camera global (once and only once)
camera = PiCamera()


class Camera():

	def __init__(self , camera):
		self.cam = camera
		self.cam.rotation = camera_rotation

	def show_scene(self, num_seconds):
		from time import sleep
		self.cam.start_preview()
		sleep(num_seconds)
		self.cam.stop_preview()

	def take_picture(self):
		pass

	def take_video(self ):
		pass

while __name__ == "__main__":
	c = Camera(camera)
	c.show_scene(num_seconds = 5)

