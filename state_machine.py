# Connor Finn
# September 18, 2020


from states.idle import Idle
from states.move import Move
from states.dance import Dance
from states.march import March
from states.lay import Lay
from states.stand import Stand
from states.standing_up import Standing_Up
from states.laying_down import Laying_Down
from states.reset_position import Reset_Position
import time


class StateMachine:

    def __init__(self, robot):
        self.state_names = ['Idle', 'Move', 'March', 'Lay', 'Dance', 'Stand',
                            'Standing_Up', 'Laying_Down', 'Reset_Position']
        #self.states = {'Idle': Idle(), 'Move': Move(), 'March': March(),
        #               'Lay': Lay(), 'Dance': Dance(), 'Stand': Stand()
        #    , 'Laying_Down': Laying_Down(), 'Standing_Up': Standing_Up(),
        #               'Reset_Position': Reset_Position()}

        self.states = {}
        for name in self.state_names:
            target_class = eval(name)
            self.states[name] = target_class()

        self.current_state = self.states['Stand']
        self.previous_state = None  # for now, just one state memory
        self.robot = robot

    def change_state(self, state_name):
        # update states
        self.previous_state = self.current_state  # keep track of previous state
        self.current_state = self.states[state_name]  # update current state
        print('switching from ', self.previous_state.__str__(), ' to ',
              self.current_state.__str__())
        self.current_state.enter(self.robot)

    def process_step(self, controller_state):
        # This will run the update step for the State
        # if a state_name is returned, that will set off the change_state method
        robot = self.robot
        state_name = self.current_state.update(robot,
                                               controller_state)  # the update step for each state will of course be different
        if state_name != None:
            self.change_state(state_name)
