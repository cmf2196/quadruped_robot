# Connor Finn
# September 19 , 2020

from state import State

class Dance(State):

	def __init__(self):
		pass


	def exit(self, new_state):
		# new_state is a string (the name of the new state to be entered)
		return new_state


	def update(self, robot ,  controller_state):
		if controller_state[6] == 1:
			return self.exit('Stand')
		else:
			return




