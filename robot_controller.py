# Connor Finn
# September 12, 2020


from PygameController import *


class robot_controller(PS4Controller):

    def __init__(self):
        super(robot_controller, self).__init__()
        

        # set maximum speeds for the robot 
        self.turbo = 0.2   # add on 20 % of current speed
        self.max_forwards_speed = 0.5 / (1 + self.turbo ) 
        self.max_backwards_speed = 0.3 / (1 + self.turbo ) 
        self.max_sideways_speed = 0.5 / (1 + self.turbo ) 
        self.max_rotation_speed = 1 / (1 + self.turbo )


        # variables for discrete only
        self.middle = 0.75
        self.walk_trot_percent = 0.5

        # What we need for our robot
        # [x_vel , y_vel , rot_vel ]
        self.robot_controller_state = [0] * 8     # modify length with number of important buttons
    
        # set the max and min robot heights
        self.max_height = - 0.2
        self.min_height = - 0.1


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
          return self.walk_trot_percent * val / abs(val)

       elif abs(val) == 0:
          return 0
       else:
          return val / abs(val)

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
    
    
    def set_robot_height(self):
        # Get the R2 pos (-1 , 1)
        try:
            pos = self.analog_state[self.analog['R2'][0]] * self.analog['R2'][1]
        except:
            pos = -1
        # convert to robot height
        self.robot_controller_state[3] = (self.max_height + self.min_height) / 2 - (self.max_height - self.min_height) * pos / 2 

    def set_height_toggle(self):
        # Check to see if the R3 button is newly being pressed 
        if self.digital_state[self.digital['R3']] == 1 and self.previous_digital_state[self.digital['R3']] == 0:
            # change from True to False and vice versa
            self.robot_controller_state[4] = 1

        else:
            self.robot_controller_state[4] = 0

    def set_march_toggle(self):
        # Check to see if the R3 button is newly being pressed 
        if self.digital_state[self.digital['L3']] == 1 and self.previous_digital_state[self.digital['L3']] == 0:
            # change from True to False and vice versa
            self.robot_controller_state[5] = 1

        else:
            self.robot_controller_state[5] = 0

    def set_dance_toggle(self):
        # Check to see if the R3 button is newly being pressed 
        if self.digital_state[self.digital['circle']] == 1 and self.previous_digital_state[self.digital['circle']] == 0:
            # change from True to False and vice versa
            self.robot_controller_state[6] = 1

        else:
            self.robot_controller_state[6] = 0

    def set_move_end_toggle(self):
        # Check to see if the R3 button is newly being pressed 
        if self.digital_state[self.digital['x']] == 1 and self.previous_digital_state[self.digital['x']] == 0:
            # change from True to False and vice versa
            self.robot_controller_state[7] = 1

        else:
            self.robot_controller_state[7] = 0



    def get_state(self , mode = 'continuous'):
        # mode is either 'continuous' , or 'discrete'
    	# refresh controller
        self.update_controller()

        # update speeds
        if mode == 'continuous':
           self.get_velocities_continuous()
        elif mode == 'discrete':
           self.get_velocities_discrete()
        
        # set the robot's height
        self.set_robot_height()

        # set toggle for standing
        self.set_height_toggle()
        # set toggle for marching
        self.set_march_toggle()
        # set toggle for dance
        self.set_dance_toggle()

        # set toggle for stoping walking
        self.set_move_end_toggle()

        
        return self.robot_controller_state

if __name__ == "__main__":

    controller = robot_controller()
  
    try:
        while (1):
            #print('analog_result ' ,controller.get_analog())
            print('speeds ' , controller.get_state(mode = 'discrete'))
            print(" ")

            time.sleep(0.5)
    except KeyboardInterrupt:
        exit()