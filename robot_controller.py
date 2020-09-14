# from PygameController_1 import PygameController , PS4Controller
# import time
# import pygame
# import platform

from PygameController_1 import *


class robot_controller(PS4Controller):

    def __init__(self):
        super(robot_controller, self).__init__()
        

        # set maximum speeds for the robot 
        self.turbo = 0.2   # add on 20 % of current speed
        self.max_forwards_speed = 0.5 / (1 + self.turbo ) 
        self.max_backwards_speed = 0.3 / (1 + self.turbo ) 
        self.max_sideways_speed = 0.5 / (1 + self.turbo ) 
        self.max_rotation_speed = 0.3 / (1 + self.turbo ) 


        # variables for discrete only
        self.middle = 0.75
        self.walk_trot_percent = 0.5

        # What we need for our robot
        # [x_vel , y_vel , rot_vel ]
        self.robot_controller_state = [0] * 3    # modify length with number of important buttons
    


    def get_velocities_continuous(self ):
        # get turbo value
        turbo_mult = 1 + self.turbo * self.digital_state[self.digital['R1']]

        # x_vel (sideways)
        self.robot_controller_state[0] = self.analog_state[self.analog['left_joystick_horizontal'][0]] * self.max_sideways_speed \
         * self.analog['left_joystick_horizontal'][1] * turbo_mult
        # y_vel (forward / backward)
        if self.analog_state[1] < 0:
        	self.robot_controller_state[1] = self.analog_state[self.analog['left_joystick_vertical'][0]] * self.max_forwards_speed \
             * self.analog['left_joystick_vertical'][1] * turbo_mult
        else:
        	self.robot_controller_state[1] = self.analog_state[self.analog['left_joystick_vertical'][0]] * self.max_backwards_speed \
             * self.analog['left_joystick_vertical'][1] * turbo_mult
        # rot_vel
        self.robot_controller_state[2] = self.analog_state[self.analog['right_joystick_horizontal'][0]] * self.max_rotation_speed \
         * self.analog['right_joystick_horizontal'][1] * turbo_mult
    

    def check_cutoffs(self , val):
       if abs(val) < self.middle and abs(val) != 0:
          return self.walk_trot_percent

       elif abs(val) == 0:
          return 0
       else:
          return 1

    def get_velocities_discrete(self ):
        # get turbo value
        turbo_mult = 1 + self.turbo * self.digital_state[self.digital['R1']]
        
        # see if below analog cutoff
        dirs = [self.analog_state[self.analog['left_joystick_horizontal'][0]] , self.analog_state[self.analog['left_joystick_vertical'][0]]  , self.analog_state[self.analog['right_joystick_horizontal'][0]] ] 
        cuttoff_percents = [self.check_cutoffs(x) for x in dirs]

        # x_vel (sideways)
        self.robot_controller_state[0] = cuttoff_percents[0] * self.max_sideways_speed * self.analog['left_joystick_horizontal'][1]  * turbo_mult
        # y_vel (forward / backward)
        if self.analog_state[1] < 0:
            self.robot_controller_state[1] = cuttoff_percents[1] * self.max_forwards_speed * self.analog['left_joystick_vertical'][1]  * turbo_mult
        else:
            self.robot_controller_state[1] = cuttoff_percents[1] * self.max_backwards_speed * self.analog['left_joystick_vertical'][1]  * turbo_mult
        # rot_vel
        self.robot_controller_state[2] = cuttoff_percents[2] * self.max_rotation_speed * self.analog['right_joystick_horizontal'][1]  * turbo_mult
    
    def update_state(self):
    	self.update_controller()

    def return_state(self , mode = 'continuous'):
        # mode is either 'continuous' , or 'discrete'
    	# refresh controller
        self.update_controller()
        if mode == 'continuous':
           self.get_velocities_continuous()
        elif mode == 'discrete':
           self.get_velocities_discrete()
        return self.robot_controller_state

if __name__ == "__main__":

    controller = robot_controller()
  
    try:
        while (1):
            print('analog_result ' ,controller.get_analog())
            print('speeds ' , controller.return_state(mode = 'discrete'))
            print(" ")

            time.sleep(0.5)
    except KeyboardInterrupt:
        exit()