
'''
This file will enable use for the ps4 controller. 
to connect the controller to the pi, follow this tutorial
https://pimylifeup.com/raspberry-pi-bluetooth/ 

This script uses the module given here https://pypi.org/project/pyPS4Controller/.
I have edited this module to allow for better control of the joysticks


the values for the joysticks are -32767 to + 32767


'''


from pyPS4Controller_edit.controller2 import Controller, Event

# Discrete speeds - one for each of the three gaits.
# two rotary speeds
# one sideways speed. 
# we could make a function here to generate a continuous speed determination, based on the joystick gradient
gait_speed = [1 , 2 , 3]  
rotation_speed = [1 , 2 ]
sideways_speed = 1

# Cutoffs to define when a gait threshoold has been crossed (note turbo button gets from trot to run)
max_threshold = 20000     # equal or greater to trot
min_threshold = 200      # equal or greater to walk

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)	
        self.joystick_state = [0 , 0 , 0]     # (sideways , forward , rotation)   
        self.turbo = False                    # This will be R1 -> used to get run speed


    def on_R1_press(self):
       self.turbo = True    # release turbo
       if self.joystick_state[1] == gait_speed[1]:
          self.joystick_state[1] = gait_speed[2]
          # Trigger new speed
          print('vertical speed set to run')
   
    def on_R1_release(self):
       self.turbo = False    # release turbo
       if self.joystick_state[1] == gait_speed[2]:
          self.joystick_state[1] = gait_speed[1]
          # Trigger new speed
          print('vertical speed set to trot')

    # Right Trigger ______________________________________

    def on_R3_at_rest(self, button_id):
       if button_id == 0 and self.joystick_state[2] != 0:
          self.joystick_state[0]=  0
          # trigger new speed!
          print(' Rotation set to zero')

    def on_R3_up(self, value):
       # Perhaps we do an upwards rotation here
       pass


    def on_R3_down(self, value):
       # Perhaps we do an downwards rotation here
       pass

    def on_R3_left(self, value):
       if abs(value)  >= max_threshold and self.joystick_state[2] != -1 * rotation_speed[1]:   #rotate quickly
          self.joystick_state[2] != rotation_speed[1]
          # Trigger new rotation speed
          print(' rotation set to fast')
  
       elif abs(value)  >= max_threshold and self.joystick_state[2] != -1 * rotation_speed[0]:   #rotate slowly
          self.joystick_state[2] != rotation_speed[0]
          # Trigger new rotation speed
          print(' rotation set to slow')
  
    def on_R3_right(self, value):
       
       if abs(value)  >= max_threshold and self.joystick_state[2] != rotation_speed[1]:   #rotate quickly
          self.joystick_state[2] != rotation_speed[1]
          # Trigger new rotation speed
          print(' rotation set to fast')
  
       elif abs(value)  >= max_threshold and self.joystick_state[2] != rotation_speed[0]:   #rotate slowly
          self.joystick_state[2] != rotation_speed[0]
          # Trigger new rotation speed
          print(' rotation set to slow')


    # Left Trigger ______________________________________

    def on_L3_at_rest(self, button_id):
       if button_id == 0 and self.joystick_state[0] != 0:
          self.joystick_state[0]=  0
          # trigger new speed!
          print(' horizontal motion set to zero ')

       elif button_id ==1 and self.joystick_state[1] != 0:
          self.joystick_state[1] = 0
          # trigger new speed
          print('vertical motion set to zero')

    def on_L3_up(self, value):
  
       if abs(value)  >= max_threshold and self.turbo == True and self.joystick_state[1] != gait_speed[2]:    # run
          self.joystick_state[1] = gait_speed[2]
          # Trigger new speed
          print(' vertical speed set to run')
  
       elif  abs(value) < max_threshold and abs(value) >= min_threshold and self.joystick_state[1] != gait_speed[0]:    # walk 
          self.joystick_state[1] = gait_speed[0]
          # Trigger new speed
          print(' vertical speed set to walk')

       elif abs(value) >= max_threshold and self.turbo == False and self.joystick_state[1] != gait_speed[1]:  # trot
          self.joystick_state[1] = gait_speed[1]
          # Trigger new Speed
          print(' vertical speed set to trot')

    def on_L3_down(self, value):

       if abs(value)  >= max_threshold and self.turbo == True and self.joystick_state[1] != -1 * gait_speed[2]:    # run
          self.joystick_state[1] = -1* gait_speed[2]
          # Trigger new speed
          print(' vertical speed set to run')
  
       elif  abs(value) < max_threshold and abs(value) >= min_threshold and self.joystick_state[1] != -1 * gait_speed[0]:    # walk 
          self.joystick_state[1] = -1 *gait_speed[0]
          # Trigger new speed
          print(' vertical speed set to walk')

       elif abs(value) >= max_threshold and self.turbo == False and self.joystick_state[1] != -1 * gait_speed[1]:  # trot
          self.joystick_state[1] = gait_speed[1]
          # Trigger new Speed
          print(' vertical speed set to trot')

    def on_L3_left(self, value):
       if abs(value) >= min_threshold and self.joystick_state[0] != -1 * sideways_speed:
          self.joystick_state[0] = sideways_speed   
  
    def on_L3_right(self, value):
       if abs(value) >= min_threshold and self.joystick_state[0] != sideways_speed:
          self.joystick_state[0] = sideways_speed 

    # stand ______________

    def on_up_arrow_press(self):
        # Trigger stand up code
        print('stand up here')

    def on_down_arrow_press(self):
        # Trigger lie down code
        print('lie down here')




    # unused buttons set to pass ________

    def on_x_press(self):
       pass

    def on_x_release(self):
       pass

    def on_triangle_press(self):
       pass

    def on_triangle_release(self):
       pass

    def on_circle_press(self):
       pass

    def on_circle_release(self):
       pass

    def on_square_press(self):
       pass

    def on_square_release(self):
       pass

    def on_L1_press(self):
       pass

    def on_L1_release(self):
       pass

    def on_L2_press(self):
       pass 

    def on_L2_release(self):
       pass 

    def on_R2_press(self):
       pass

    def on_R2_release(self):
       pass

    def on_up_down_arrow_release(self):
       pass

    def on_left_arrow_press(self):
       pass

    def on_left_right_arrow_release(self):
       pass

    def on_right_arrow_press(self):
       pass

    def on_L3_press(self):
       pass 

    def on_L3_release(self):
       pass 

    def on_R3_press(self):
       pass 

    def on_R3_release(self):
       pass 

    def on_options_press(self):
       pass 

    def on_options_release(self):
       pass

    def on_share_press(self):
       pass 

    def on_share_release(self):
       pass 

    def on_playstation_button_press(self):
       pass 

    def on_playstation_button_release(self):
       pass 



class MyEventDefinition(Event):

    def __init__(self, **kwargs):
        Event.__init__(self, **kwargs)

    # each overloaded function, has access to:
    # - self.button_id
    # - self.button_type
    # - self.value
    # use those variables to determine which button is being pressed

    def x_pressed(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 1

    def x_released(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 0


    def circle_pressed(self):
        return self.button_id == 1 and self.button_type == 1 and self.value == 1

    def circle_released(self):
        return self.button_id == 1 and self.button_type == 1 and self.value == 0

    def square_pressed(self):
        return self.button_id == 3 and self.button_type == 1 and self.value == 1

    def square_released(self):
        return self.button_id == 3 and self.button_type == 1 and self.value == 0

    def triangle_pressed(self):
        return self.button_id == 2 and self.button_type == 1 and self.value == 1

    def triangle_released(self):
        return self.button_id == 2 and self.button_type == 1 and self.value == 0
  
    def R3_at_rest(self ):

        return self.button_id in [3 , 4 ] and self.button_type == 2 and self.value == 0 

    def L3_at_rest(self ):

        return self.button_id in [0 , 1 ] and self.button_type == 2 and self.value == 0


controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False, event_definition=MyEventDefinition)
#controller.debug = True  # you will see raw data stream for any button press, even if that button is not mapped
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
