# Connor Finn
# September 19 , 2020

from state import State

class Lay(State):

	def __init__(self):
		pass


	def exit(self , new_state):
		print(' we got here !')
		# new_state is a string (the name of the new state to be entered)
		return new_state


	def update(self, robot , controller_state):
		# all we need to do here is wait to stand up
		#print('now in lay')
		if controller_state[4] == 1:
			return self.exit('idle')
		else:
			return





