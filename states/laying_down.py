# Connor Finn
# September 19 , 2020

from state import State

class Laying_Down(State):
	def __init__(self):
		self.clock_max = 0
		self.clock = 0
		self.lay_speed = .1  # m / second

	
	
	def get_points(self , robot ):
		# in this function we simply cancel units  m / (m/s) * (pts / s)
		dist = robot.trajectory_executor.laying_position[0][2] - robot.trajectory_executor.current_position[0][2] 
		points = ( dist / self.lay_speed ) *  robot.trajectory_executor.leg_trajectory_generator.frequency

		return int(points )


	def enter(self , robot):
		# comute the trajectory we want to follow
		robot.trajectory_executor.move_index = 0
		robot.trajectory_executor.leg_single_trajectory = []   
		# Get amount of points used to lay down 
		num_points =  self.get_points(robot)
		l = 0
		# calculate the trajectory for each foot for this move
		for leg in robot.trajectory_executor.default_pose:
			traj = robot.trajectory_executor.leg_trajectory_generator.compute_leg_linear_trajectory2(robot.trajectory_executor.current_position[l],\
		 		robot.trajectory_executor.laying_position[l], num_points + 1)
			robot.trajectory_executor.leg_single_trajectory.append(list(zip(traj[0], traj[1], traj[2])))
			l += 1 

		# WITHOUT TRANSITIONS, THIS WILL RESULT IN DISCONTINUITY!
		robot.trajectory_executor.modes = ["leg_single_trajectory", "leg_single_trajectory", "leg_single_trajectory", "leg_single_trajectory"]
		robot.trajectory_executor.clock_max = len(robot.trajectory_executor.leg_single_trajectory[0]) - 1
		self.clock_max = len(robot.trajectory_executor.leg_single_trajectory[0]) - 1

		return 

	def exit(self , new_state):
		# new_state is a string (the name of the new state to be entered)
		self.clock = 0
		return new_state


	def update(self, robot , controller_state):
		if robot.imu_state != ["stable", "stable"]:
			return self.exit('Fallen')

		# cancels the process and starts standing back up
		if controller_state[4] == 1:
			return self.exit('Standing_Up')

		# Reached the bottom, go to Lay state
		if self.clock == self.clock_max:
			robot.trajectory_executor.modes = ["idle", "idle", "idle", "idle"]
			return self.exit('Lay')
		
		# get the command 
		command = robot.trajectory_executor.get_next_command()
		# get IK
		ik = robot.simulator.compute_multi_ik(robot.feet, command)

		# if simulating, move simulation
		if robot.simulator.gui:
			robot.simulator.set_robot_pos(robot.moving_joints, ik)
			robot.simulator.step_gui_sim()
			robot.simulator.center_camera()

		# move motors 
		if robot.motors:
			degs = robot.motor_controller.radians_to_degrees(ik)

			robot.motor_controller.move_all_motors(degs, int(robot.period*1000))

		# update the clock
		self.clock += 1		