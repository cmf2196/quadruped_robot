

from robot_controller import *
from state_machine import *
import time



def sleep_until_next_cycle( start_time, end_time, time_step):
    difference = end_time - start_time
    if end_time - start_time < time_step:
        time.sleep(time_step - difference)
    else:
        print("Overtime!")



if __name__ == '__main__':
	sm = StateMachine()
	controller = robot_controller()
	
	robot = 'robot'  # place holder for now


	x = 0
	while (1):
		x += 1
		# record start time
		start_time = time.time()
		# check controller
		# velocity = self.get_keyboard_command()
		controller_command = controller.get_state(mode = 'discrete')	
		sm.process_step(robot , controller_command)
		# sleep until next cycle
		end_time = time.time()

		sleep_until_next_cycle(start_time, end_time, .01)
