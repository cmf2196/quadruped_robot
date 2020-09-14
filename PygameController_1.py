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
        self.num_analog = self.joystick.get_numaxes()
        self.num_digital = self.joystick.get_numbuttons()
        # keep a running tab of the controller state
        self.digital_state = [0] * self.num_digital
        self.analog_state = [0] * self.num_analog

        # for analog control
        self.minimum = 0.2


    def connect_to_controller(self):

        # check if controller is plugged in
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            # No joysticks!
            print("Error, I didn't find any joysticks.")
            exit()

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
        self.digital_state = [self.joystick.get_button(v) for v in d_vals]

    def update_analog(self):
        a_vals = range(self.num_analog)
        states = [self.joystick.get_axis(v) for v in a_vals]
        self.analog_state = [self.check_min(s) for s in states]


   
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


    def get_analog(self):
        self.update_controller()
        return self.analog_state

    def get_digital(self):
        self.update_controller()
        return self.digital_state

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
            


        elif platform.system() == 'Linux':  
            self.digital = {'x' : 0 , 'circle': 1 , 'triangle':2  , 'square': 3 , 'L1': 4 , 'R1': 5 , 'share': 11 , 'options': 12 \
              , 'power': 13 , 'L3':  14 , 'R3': 15 }
            self.analog = {'left_joystick_horizontal': [0 , 1] , 'left_joystick_vertical': [1 , -1 ] , 'L2': [2 , 1] , 'right_joystick_horizontal': [3 , 1] \
              , 'right_joystick_vertical': [4 , -1]  , 'R2': [5 , 1]}

        # JOSH - Run pygame_config.py and figure out your button mapping
        elif platform.system() == 'Windows':  
            self.digital = {'x' : 0 , 'circle': 1 , 'square':2  , 'triangle': 3 , 'share': 4 , 'power': 5 , 'options': 6 , 'L3': 7 \
              , 'R3': 8 , 'L1':  9 , 'R1': 10 , 'up_arrow': 11 , 'down_arrow': 12 , 'left_arrow': 13 , 'right_arrow' : 14 , 'touchpad': 15}
            self.analog = {'left_joystick_horizontal': [0 , 1] , 'left_joystick_vertical': [1 , 1 ] , 'right_joystick_horizontal': [2 , 1] \
              , 'right_joystick_vertical': [3 , 1] , 'L2': [4 , 1] , 'R2': [5 , 1]}


