'''
Joshua Katz
9/8/20
'''

import time
import os
import keyboard
import platform

from Simulator import Simulator
from TrajectoryExecutor import TrajectoryExecutor

from ps4_controller import MyController
from ps4_controller import MyEventDefinition


class Robot:

    def __init__(self, urdf, gui=False):
        self.urdf = urdf
        self.frequency = 100  # Hz
        self.period = 1 / self.frequency

        # from urdf, determine moving joints (at some point possibly)
        self.moving_joints = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        self.feet = [3, 7, 11, 15]

        # initialize simulator
        self.simulator = Simulator(gui)
        self.gui = gui
        if self.gui:
            self.simulator.load_gui_urdf(urdf)
        self.simulator.load_kinematics_urdf(urdf)

        # initialize trajectory executor
        self.trajectory_executor = TrajectoryExecutor()

        # initialize controller connection
        self.controller = MyController(interface="/dev/input/js0",
                                       connecting_using_ds4drv=False,
                                       event_definition=MyEventDefinition)
        self.controller.initialize_connection()

        # execute orient and stand up sequence
        # function???

        # make velocity 0
        self.trajectory_executor.current_position = [(-0.135, 0.15, -0.2),
                                                     (0.135, 0.15, -0.2),
                                                     (-0.135, -0.15, -0.2),
                                                     (0.135, -0.15, -0.2)]
        self.trajectory_executor.change_movement_speed(0, 0.1,
                                                       0)  # makes cycles exist
        self.trajectory_executor.change_movement_speed(0, 0, 0)

    def get_keyboard_command(self):
        x, y, a = (0, 0, 0)

        if keyboard.is_pressed("up"):
            y = 0.3
        if keyboard.is_pressed("left"):
            x = -0.2
        if keyboard.is_pressed("down"):
            y = -0.2
        if keyboard.is_pressed("right"):
            x = 0.2
        if keyboard.is_pressed("q"):
            a = 1
        if keyboard.is_pressed("e"):
            a = -1

        return x, y, a

    def get_controller_command(self):
        return self.controller.get_state()

    def sleep_until_next_cycle(self, start_time, end_time, time_step):
        difference = end_time - start_time
        if end_time - start_time < time_step:
            time.sleep(time_step - difference)
        else:
            print("Overtime!")

    def main_loop(self):

        # Have controller start listening
        #        self.controller.listen(timeout=60)
        #        print('here')
        while (1):
            # record start time
            start_time = time.time()

            # check controller
            # velocity = self.get_keyboard_command()
            velocity = self.get_controller_command()

            print(velocity)
            # check orientation

            # update and check state

            # calculate/ look up new joint positions
            self.trajectory_executor.change_movement_speed(velocity[0],
                                                           velocity[1],
                                                           velocity[2])

            command = self.trajectory_executor.get_next_command()

            # move motors
            ik = self.simulator.compute_multi_ik(self.feet, command)

            # if simulating, move simulation
            if self.simulator.gui:
                self.simulator.set_robot_pos(self.moving_joints, ik)
                self.simulator.step_gui_sim()
                self.simulator.center_camera()

            # sleep until next cycle
            end_time = time.time()

            self.sleep_until_next_cycle(start_time, end_time, self.period)


if __name__ == "__main__":
    # select urdf
    current_dir = os.getcwd()
    sep = os.path.sep
    urdf = current_dir + sep + "Phantom" + sep + "urdf" + sep + "Phantom_connor_edits.urdf"

    # Create robot object and run its main loop

    # if on linux, do not show gui
    if platform.system() == "Linux":
        gui = False
    else:
        gui = True

    robot = Robot(urdf, gui)

    robot.main_loop()
