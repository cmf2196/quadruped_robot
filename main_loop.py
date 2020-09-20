from state_machine import *
from Robot import *
import time 


if __name__ == '__main__':
    sm = StateMachine()
    controller = robot_controller()
    
    current_dir = os.getcwd()
    sep = os.path.sep
    urdf = current_dir + sep + "Phantom" + sep + "urdf" + sep + "Phantom_connor_edits.urdf"

    # Create robot object and run its main loop

    # if on linux, do not show gui
    if platform.system() == "Linux":
        gui = False
    else:
        gui = True

    if platform.system() == "Linux":
        robot = Robot(urdf, gui, True)
    else:
        robot = Robot(urdf, gui, False)
    
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

        robot.sleep_until_next_cycle(start_time, end_time, robot.period)


