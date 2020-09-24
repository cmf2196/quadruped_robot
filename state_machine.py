# Connor Finn
# September 18, 2020


from states.idle import Idle 
from states.move import Move 
from states.dance import Dance 
from states.march import March
from states.lay import Lay 
from states.stand import Stand
import time




class StateMachine:

	def __init__(self , robot):
		self.state_names = ['Idle' , 'Move' , 'March' , 'Lay' , 'Dance' , 'Stand']
		self.states = {'Idle': Idle() , 'Move': Move() , 'March': March() , 'Lay': Lay() , 'Dance': Dance() , 'Stand': Stand()}
		self.current_state = self.states['Stand'] 
		self.previous_state = None       # for now, just one state memory
		self.robot = robot


	def change_state(self , state_name):
		if self.current_state.__str__() == 'Stand' and state_name == 'Lay':
			self.stand_to_lay()
		
		elif self.current_state.__str__() == 'Lay' and state_name == 'Stand':
			self.stand_up()

		elif self.current_state.__str__() in ['Idle' , 'Dance' , 'March'] and state_name == 'Stand':
			self.reset_feet()
		

		# update states
		self.previous_state = self.current_state      # keep track of previous state
		self.current_state = self.states[state_name]  # update current state
		print('switching from ' , self.previous_state.__str__() , ' to ' , self.current_state.__str__())
	

	def stand_up(self):
		print(" in stand_up function")
		robot = self.robot
		robot.trajectory_executor.mode = "standing"

		while robot.trajectory_executor.mode == "standing":
			# calculate/ look up new joint positions
			start_time = time.time()
			robot.trajectory_executor.change_movement_speed(0, 0 , 0)
			command = robot.trajectory_executor.get_next_command()
			print(command)
			# move motors
			ik = robot.simulator.compute_multi_ik(robot.feet, command)

			# if simulating, move simulation
			if robot.simulator.gui:
				robot.simulator.set_robot_pos(robot.moving_joints, ik)
				robot.simulator.step_gui_sim()
				robot.simulator.center_camera()

			if robot.motors:
				degs = robot.motor_controller.radians_to_degrees(ik)

				robot.motor_controller.move_all_motors(degs, int(robot.period*1000))
			end_time = time.time()
			robot.sleep_until_next_cycle(start_time, end_time, robot.period)

	def reset_feet(self):
		robot = self.robot
		return

	def stand_to_lay(self):
		print('go to laying position')
		robot = self.robot
		robot.trajectory_executor.mode = "laying"

		while robot.trajectory_executor.mode == "laying":
			# calculate/ look up new joint positions
			start_time = time.time()
			robot.trajectory_executor.change_movement_speed(0, 0 , 0)
			command = robot.trajectory_executor.get_next_command()
			print('command')
			# move motors
			ik = robot.simulator.compute_multi_ik(robot.feet, command)

			# if simulating, move simulation
			if robot.simulator.gui:
				robot.simulator.set_robot_pos(robot.moving_joints, ik)
				robot.simulator.step_gui_sim()
				robot.simulator.center_camera()

			if robot.motors:
				degs = robot.motor_controller.radians_to_degrees(ik)

				robot.motor_controller.move_all_motors(degs, int(robot.period*1000))
			end_time = time.time()
			robot.sleep_until_next_cycle(start_time, end_time, robot.period)



	def process_step(self , controller_state):
		# This will run the update step for the State
		# if a state_name is returned, that will set off the change_state method
		robot = self.robot 
		state_name = self.current_state.update(robot , controller_state)     # the update step for each state will of course be different
		if state_name != None:
			self.change_state(state_name)




