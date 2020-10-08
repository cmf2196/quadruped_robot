# Connor Finn
# September 19 , 2020

from state import State

class Lay(State):

	def __init__(self):
		pass

	def enter(self, robot):
		# turn off motors
	    robot.motor_controller.turn_off_all_motors()


	def exit(self , new_state):
		# new_state is a string (the name of the new state to be entered)

		# upon exit, turn motors back on


		return new_state


	def update(self, robot , controller_state):
		# all we need to do here is wait to stand up
		#print('now in lay')
		if controller_state[4] == 1:
			robot.motor_controller.turn_on_all_motors()
			return self.exit('Standing_Up')
		else:
			return





