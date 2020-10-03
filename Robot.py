'''
Joshua Katz
9/8/20
'''

import time
import os
import platform
from Simulator import Simulator
from TrajectoryExecutor import TrajectoryExecutor
from state_machine import *

from robot_controller import *
from ps4_controller import MyController
from ps4_controller import MyEventDefinition

from PygameController import PygameController

if platform.system() == "Linux":
    from MotorController import MotorController
    from imu import *
else:
    import keyboard

class Robot:

    def __init__(self, urdf, gui=False, motors = False):
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

        self.controller = robot_controller()
        self.state_machine = StateMachine(self)

        # if platform.system() == "Linux":
        #     self.controller = MyController(interface="/dev/input/js0",
        #                                    connecting_using_ds4drv=False,
        #                                    event_definition=MyEventDefinition)
        #     self.controller.initialize_connection()
        # else:
        #     self.controller = PygameController()

        # make velocity 0, place in standing position virtually
        #self.trajectory_executor.current_position = [(-0.135, 0.15, -0.2),
        #                                             (0.135, 0.15, -0.2),
        #                                             (-0.135, -0.15, -0.2),
        #                                             (0.135, -0.15, -0.2)]
        self.trajectory_executor.change_movement_speed(0, 0.1,
                                                       0)  # makes cycles exist
        self.trajectory_executor.change_movement_speed(0, 0, 0)

        # execute orient and stand up sequence
        # For now, I will hard code a stand up sequence. This will
        # be replaced by FSM later...

        # initialize the IMU sensor
        if platform.system() == "Linux":
            self.imu = IMU()


        self.motors = motors
        self.offsets = [0 , 0 , 0 , 0 , 0 , 0, 0, 0, 0, 0, 0, 0]
        if self.motors:
            self.motor_controller = MotorController(
                [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42,
                 43], self.frequency , self.offsets)
            lay = [(-0.135, 0.15, -0.03),
                   (0.135, 0.15, -0.03),
                   (-0.135, -0.15, -0.03),
                   (0.135, -0.15, -0.03)]
            ik_lay = self.simulator.compute_multi_ik(self.feet, lay)
            self.motor_controller.move_all_motors(self.motor_controller.radians_to_degrees(ik_lay),2000)
            time.sleep(2)
            ik_stand = self.simulator.compute_multi_ik(self.feet, self.trajectory_executor.current_position)

            # iterate to make stand position better (pybullet sucks at this...)
            # This prevents a sudden change in position once cycles begin
            for x in range(0, 10):
                ik_stand = self.simulator.compute_multi_ik(self.feet,
                                                           self.trajectory_executor.current_position)
            self.motor_controller.move_all_motors(
                self.motor_controller.radians_to_degrees(ik_stand), 2000)
            time.sleep(3)

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
        return self.controller.get_state(mode = 'discrete')

        # if type(self.controller).__name__ == "MyController":
        #     return self.controller.get_state()
        #     print('state read')
        # else:
        #     return self.controller.multi_axis_to_velocity()

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

        x = 0
        while (1):
            x += 1
            # record start time
            start_time = time.time()

            # check controller
            # velocity = self.get_keyboard_command()
            controller_command = self.get_controller_command()

            if platform.system() == "Linux":
                # update the imu sesnor
                self.imu.update_state()

            self.state_machine.process_step(controller_command)
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

    if platform.system() == "Linux":
        robot = Robot(urdf, gui, True)
    else:
        robot = Robot(urdf, gui, False)
    robot.main_loop()
