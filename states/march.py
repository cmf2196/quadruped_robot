# Connor Finn
# September 19 , 2020

from state import State

class March(State):

	def __init__(self):
		pass


	def exit(self , new_state):
		# new_state is a string (the name of the new state to be entered)
		return new_state


	def update(self, robot , controller_state):
		if controller_state[5] == 1:
			return self.exit('idle')
		elif [0 , 0 , 0] != controller_state[:3]:
			return self.exit('move')
		else:
			return




