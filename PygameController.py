"""
Connor Finn
9/14/2020

# To run.
    + You need pygame 2 python3 -m pip install pygame==2.0.0.dev6
    + Connect your PS4 Controller to the computer over bluetooth (wired will work fine too)

# This is a modified Version of Josh's PygameController
"""

import time
import pygame
import platform


class PygameController:

    def __init__(self):
        self.joystick = None
        pygame.init()
        self.connect_to_controller()
        
        if self.joystick != None:
            self.num_analog = self.joystick.get_numaxes()
            self.num_digital = self.joystick.get_numbuttons()
            self.num_hat = self.joystick.get_numhats()

            # keep a running tab of the controller state
            self.digital_state = [0] * self.num_digital
            self.analog_state = [0] * self.num_analog
            self.hat_state = [0] * self.num_hat

        # i want the previous digital state as well so that we  can
        # keep track of changes
        self.previous_digital_state = None

        # for analog control
        self.minimum = 0.2


    def connect_to_controller(self):

        # check if controller is plugged in
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            # No joysticks!
            print("Error, I didn't find any joysticks.")
            self.joystick = None

        else:
            # Use joystick #0 and initialize it
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            # Prints the joystick's name
            JoyName = pygame.joystick.Joystick(0).get_name()
            print(JoyName)


    def get_button(self, button):
        self.refresh_controller()
        return self.joystick.get_button(button)

    def update_digital(self):
        d_vals = range(self.num_digital)
        self.previous_digital_state = self.digital_state
        self.digital_state = [self.joystick.get_button(v) for v in d_vals]

    def update_analog(self):
        a_vals = range(self.num_analog)
        states = [self.joystick.get_axis(v) for v in a_vals]
        self.analog_state = [self.check_min(s) for s in states]


    def update_hat(self):
        h_vals = range(self.num_hat)
        self.hat_state = [self.joystick.get_hat(h) for h in h_vals]

    def check_min(self, val):
        # don't want to have 0.2 or less


        if abs(val) <= self.minimum:
            return 0
        else:
            return val


    def update_controller(self):
        # get current events
        self.refresh_controller()
        # update buttons
        self.update_analog()
        self.update_digital()
        self.update_hat()


    def get_analog(self):
        self.update_controller()
        return self.analog_state

    def get_digital(self):
        self.update_controller()
        return self.digital_state

    def get_hat(self):
        self.update_controller()
        return self.hat_state

    @staticmethod
    def refresh_controller():
        # This is called each time to get the current state of the controller
        pygame.event.get()



class PS4Controller(PygameController):

    def __init__(self):
        super(PS4Controller, self).__init__()

        if platform.system() == 'Darwin':
            self.digital = {'x' : 0 , 'circle': 1 , 'square':2  , 'triangle': 3 , 'share': 4 , 'power': 5 , 'options': 6 , 'L3': 7 \
              , 'R3': 8 , 'L1':  9 , 'R1': 10 , 'up_arrow': 11 , 'down_arrow': 12 , 'left_arrow': 13 , 'right_arrow' : 14 , 'touchpad': 15}
            # values are (id , dir) id is int, dir is -1 or 1 (do the values need to be flipped)
            # R2, L2 should be -1 when not used, 1 when used
            # for joysticks, left and down are -1 , up and right are 1
            self.analog = {'left_joystick_horizontal': [0 , 1] , 'left_joystick_vertical': [1 , -1 ] , 'right_joystick_horizontal': [2 , 1] \
              , 'right_joystick_vertical': [3 , -1] , 'L2': [4 , 1] , 'R2': [5 , 1]}

            self.hat = {}


        elif platform.system() == 'Linux':
            self.digital = {'x' : 0 , 'circle': 1 , 'triangle':2  , 'square': 3 , 'L1': 4 , 'R1': 5 , 'share': 8 , 'options': 9 \
              , 'power': 10 , 'L3':  11 , 'R3': 12 }
            self.analog = {'left_joystick_horizontal': [0 , 1] , 'left_joystick_vertical': [1 , 1 ] , 'L2': [2 , 1] , 'right_joystick_horizontal': [3 , 1] \
              , 'right_joystick_vertical': [4 , 1]  , 'R2': [5 , 1]}
            self.hat = {}

        # JOSH - Run pygame_config.py and figure out your button mapping
        elif platform.system() == 'Windows':
            self.digital = {'x' : 1 , 'circle': 2 , 'square':0  , 'triangle': 3 , 'share': 8 , 'power': 12 , 'options': 9 , 'L3': 10 \
              , 'R3': 11 , 'L1':  4 , 'R1': 5 , 'touchpad': 13}
            self.analog = {'left_joystick_horizontal': [0 , 1] , 'left_joystick_vertical': [1 , -1 ] , 'right_joystick_horizontal': [2 , 1] \
              , 'right_joystick_vertical': [3 , -1] , 'L2': [4 , 1] , 'R2': [5 , 1]}

            self.hat = {'none' : (0,0), 'left': (-1,0), 'up': (0,1),'right': (1,0),
                        'down': (0,-1),'up_left': (-1,1),'up_right': (1,1),
                        'down_right': (1,-1),'down_left': (-1,-1),}


if __name__ == '__main__':
    controller = PygameController()

    while True:
        print(controller.get_hat())
        time.sleep(0.5)