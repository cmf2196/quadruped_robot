# Connor Finn
# September 19 , 2020

from state import State

class Lay(State):

	def __init__(self):
		pass


	
	def enter(self , robot):
		if robot.motors:
			robot.motor_controller.turn_all_motors_off() 


	def exit(self , new_state , robot):
		if robot.motors:
			robot.motor_controller.turn_all_motors_on()
		return new_state


	def update(self, robot , controller_state):
		if robot.imu_state != ["stable", "stable"]:
			return self.exit('Fallen')

		# all we need to do here is wait to stand up
		#print('now in lay')
		if controller_state[4] == 1:
			return self.exit('Standing_Up' , robot)
		else:
			return





