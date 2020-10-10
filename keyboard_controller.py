# Connor Finn
# October 4, 2020

# I want to make the keyboard into its own class. That way we can easily match it to the ps4 controller
import keyboard



class Keyboard_Controller():



	def __init__(self):
		self.state = [0] * 8
		self.previous_state = [0] * 8


	def update_keyboard(self):
		self.previous_state = self.state.copy()

		# move left
		if keyboard.is_pressed("left"):
			self.state[0] = -0.2
		# move right
		elif keyboard.is_pressed("right"):
			self.state[0] = -0.2
		else:
			self.state[0] = 0

		# move forward
		if keyboard.is_pressed("up"):     
			self.state[1] = 0.2
		# move backwards
		elif keyboard.is_pressed("down"):
			self.state[1] = -0.2
		else:
			self.state[1] = 0

		# turn right
		if keyboard.is_pressed("q"):
			self.state[2] = 1
		# turn left
		elif keyboard.is_pressed("e"):
			self.state[2] = -1
		else:
			self.state[2] = 0
		# stand toggle     
		if keyboard.is_pressed("z"):
			self.state[4] = 1

		else:
			self.state[4] = 0
		# march toggle
		if keyboard.is_pressed("s"):
			self.state[5] = 1

		else:
			self.state[5] = 0
		# dance toggle
		if keyboard.is_pressed("c"):
			self.state[6] = 1

		else:
			self.state[6] = 0
		# reset button
		if keyboard.is_pressed("x"):
			self.state[7] = 1
		else:
			self.state[7] = 0




	def get_command(self):
		self.update_keyboard()
		command = self.state.copy() 
		# stand toggle     
		if self.state[4] == 1 and self.previous_state[4] != 1:
			command[4] = 1

		else:
			command[4] = 0


		# stand toggle     
		if self.state[5] == 1 and self.previous_state[5] != 1:
			command[5] = 1

		else:
			command[5] = 0
		
		# stand toggle     
		if self.state[6] == 1 and self.previous_state[6] != 1:
			command[6] = 1

		else:
			command[6] = 0

		print('previous ' , self.previous_state)
		print('command ' , command)
		return command 






