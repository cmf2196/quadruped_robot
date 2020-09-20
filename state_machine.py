# Connor Finn
# September 18, 2020


from states.idle import Idle 
from states.move import Move 
from states.dance import Dance 
from states.march import March
from states.lay import Lay 




class StateMachine:

	def __init__(self):
		self.state_names = ['idle' , 'move' , 'march' , 'lay' , 'dance']
		self.states = {'idle': Idle() , 'move': Move() , 'march': March() , 'lay': Lay() , 'dance': Dance()}
		self.current_state = self.states['idle'] 
		self.previous_state = None       # for now, just one state memory


	def change_state(self , state_name):
		#if state_name == 'previous'
		self.previous_state = self.current_state      # keep track of previous state
		self.current_state = self.states[state_name]  # update current state
		print('switching from ' , self.previous_state.__str__() , ' to ' , self.current_state.__str__())
	def process_step(self , robot , controller_state):
		# This will run the update step for the State
		# if a state_name is returned, that will set off the change_state method
		state_name = self.current_state.update(robot , controller_state)     # the update step for each state will of course be different
		if state_name != None:
			self.change_state(state_name)




