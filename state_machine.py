'''
Authors: Connor Finn, Josh Katz 
Summer 2020

Description:

    This script dictates a finate state machine that will be used to control he robot's actions and 
    transitions between different operating modes. Each state exists as a separate file in the states directory.
    These states each have their own update step, as well as enter and exit functions which describe how the robot
    will operate while within this state or while transitioning in or out of the state. The specific motions of the 
    robot are determined in the TrajectoryExecutor and LegTrajetoryGenorator files. The os currently assumes 100 Hz

    Transitions:
        Transitions between states are envoked through a PS4 controller. The controller commands are indicated in 
        robot_controller.py. It is possible to use a different controller, However, it will be necessary to update 
        the settings in both the robot_controller and PygameController files.

        Currently: 
            + R3 Toggles between Laying and Standing
            + x is used to reset the feet while standing
            + x is also used to recover when in a fallen position
            + L3 starts a march
            + the Left Joystick indicates the direction and speed of motion 
            + R1 is the turbo button
            + The right Joystick indicates the turning direction and speed

'''

from states.state import State
from states.idle import Idle
from states.move import Move
from states.dance import Dance
from states.march import March
from states.lay import Lay
from states.stand import Stand
from states.standing_up import Standing_Up
from states.laying_down import Laying_Down
from states.reset_position import Reset_Position
from states.recovering import Recovering
from states.fallen import Fallen
import time


class StateMachine:

    def __init__(self, robot):
        self.state_names = ['Idle', 'Move', 'March', 'Lay', 'Dance', 'Stand',
                            'Standing_Up', 'Laying_Down', 'Reset_Position' , 'Fallen' , 'Recovering']
        # create a dictionary of State objects
        self.states = {}
        for name in self.state_names:
            target_class = eval(name)
            self.states[name] = target_class()

        self.current_state = self.states['Stand']
        self.previous_state = None 
        self.robot = robot

    def change_state(self, state_name):
        # This function transitions the robot into a new state

        # set the current state as the previous state
        self.previous_state = self.current_state 
        # update the current state to the indicated state        
        self.current_state = self.states[state_name] 
        # Run the enter function for the new state
        self.current_state.enter(self.robot)

    def process_step(self, controller_state):
        robot = self.robot
        # run the update step
        state_name = self.current_state.update(robot, controller_state)
        # change the state if a new name is returned
        if state_name != None:
            self.change_state(state_name)



