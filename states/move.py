# Connor Finn
# September 19 , 2020

from state import State

class Move(State):

	def __init__(self):
		pass


	def exit(self , new_state):
		# new_state is a string (the name of the new state to be entered)
		return new_state


	def update(self, robot , controller_state):
		if robot.imu_state[0] != "Stable" or robot.imu_state[1] != "Stable":
			return self.exit('Fallen')

		if [abs(a) for a in controller_state[:3]] == [0 , 0 , 0]:
			return self.exit('Idle')

		elif controller_state[7] == 1:
			return self.exit('Stand')

		# calculate/ look up new joint positions
		robot.trajectory_executor.change_movement_speed(controller_state[0],
                                                               controller_state[1],
                                                               -1 * controller_state[2])

		command = robot.trajectory_executor.get_next_command()

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





