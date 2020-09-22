# Connor Finn
# September 18, 2020


from states.idle import Idle 
from states.move import Move 
from states.dance import Dance 
from states.march import March
from states.lay import Lay 
from states.stand import Stand




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
		
		elif state_name == 'Stand' or self.current_state.__str__() =='Idle' and state_name in ['Dance' , 'March']:
			self.to_stand()

		elif self.current_state.__str__() == 'idle' and state_name == 'Lay' :
			self.to_stand()
			self.stand_to_lay()

		# update states
		self.previous_state = self.current_state      # keep track of previous state
		self.current_state = self.states[state_name]  # update current state
		print('switching from ' , self.previous_state.__str__() , ' to ' , self.current_state.__str__())
	

	def to_stand(self):
		print('go to standing position')
		robot = self.robot
		stand = robot.trajectory_executor.stand_position

		# iterate to make stand position better (pybullet sucks at this...)
		# This prevents a sudden change in position once cycles begin
		ik_stand = robot.simulator.compute_multi_ik(robot.feet, stand)

		# iterate to make stand position better (pybullet sucks at this...)
		# This prevents a sudden change in position once cycles begin
		for x in range(0, 10):
			ik_stand = robot.simulator.compute_multi_ik(robot.feet,
                                                           stand)    
		# if simulating, move simulation
		if robot.simulator.gui:
			robot.simulator.set_robot_pos(robot.moving_joints, ik_stand)
			robot.simulator.step_gui_sim()
			robot.simulator.center_camera()
                

		if robot.motors:
			robot.motor_controller.move_all_motors(robot.motor_controller.radians_to_degrees(ik_stand),2000)

	def stand_to_lay(self):
		print('go to laying position')
		robot = self.robot
		lay = [(-0.135, 0.15, -0.03),
			(0.135, 0.15, -0.03),
			(-0.135, -0.15, -0.03),
			(0.135, -0.15, -0.03)]
		ik_lay = robot.simulator.compute_multi_ik(robot.feet, lay)

		# if simulating, move simulation
		if robot.simulator.gui:
			robot.simulator.set_robot_pos(robot.moving_joints, ik_lay)
			robot.simulator.step_gui_sim()
			robot.simulator.center_camera()
                

		if robot.motors:
			robot.motor_controller.move_all_motors(robot.motor_controller.radians_to_degrees(ik_lay),2000)





	def process_step(self , controller_state):
		# This will run the update step for the State
		# if a state_name is returned, that will set off the change_state method
		robot = self.robot 
		state_name = self.current_state.update(robot , controller_state)     # the update step for each state will of course be different
		if state_name != None:
			self.change_state(state_name)




