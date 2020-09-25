

from state import State

class Reset_Position(State):

	def __init__(self):
		self.clock_max = 0
		self.clock = 0


	def enter(self , robot):
		# comute the trajectory we want to follow


	def exit(self , new_state):
		# new_state is a string (the name of the new state to be entered)
		return new_state


	def update(self, robot , controller_state):

		# finished the process go to Stand
		if self.clock == self.clock_max:
			self.clock = 0
			robot.trajectory_executor.modes = ["idle", "idle", "idle", "idle"]
			return self.exit('Stand')

		# get the current command
		command = robot.trajectory_executor.get_next_command()

		# get IK
		ik = robot.simulator.compute_multi_ik(robot.feet, command)

		# if simulating, move simulation
		if robot.simulator.gui:
			robot.simulator.set_robot_pos(robot.moving_joints, ik)
			robot.simulator.step_gui_sim()
			robot.simulator.center_camera()

		# move robot
		if robot.motors:
			degs = robot.motor_controller.radians_to_degrees(ik)

			robot.motor_controller.move_all_motors(degs, int(robot.period*1000))

		# update the clock
		self.clock += 1		