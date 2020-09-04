
'''
This file will enable use for the ps4 controller. 
to connect the controller to the pi, follow this tutorial
https://pimylifeup.com/raspberry-pi-bluetooth/ 

This script uses the module given here https://pypi.org/project/pyPS4Controller/.
I have edited this module to allow for better control of the joysticks


the values for the joysticks are -32767 to + 32767

'''


from pyPS4Controller_edit.controller2 import Controller, Event


max_sideways_speed = 6.0    # m/s
max_forward_speed = 10.0   # m/s
max_backward_speed = -4.0    # m/s
max_rotation_speed = 2      # rad/s

speed_percentages = [0.2 , 0.6 , 1]  # walk, trot , run  note, rotation just used the First two!

# Cutoffs to define when a gait threshoold has been crossed (note turbo button gets from trot to run)
max_threshold = 25000     # equal or greater to trot
min_threshold = 10000      # equal or greater to walk

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)	
        self.joystick_state = [0 , 0 , 0]     # (sideways , forward , rotation)   
        self.turbo = False                    # This will be R1 -> used to get run speed


    def get_speed(self , val):
       if val >= min_threshold and val < max_threshold:    # low speed
          return speed_percentages[0]     

       elif val >= max_threshold and self.turbo == False:  # medium speed
          return speed_percentages[1]

       elif val >= max_threshold and self.turbo == True:  # meax speed
          return speed_percentages[2]

       else:          # signal too small
          # this does NOT mean at rest, it means do not change current state
          return 0   

    def on_R1_press(self):
       self.turbo = True    # release turbo

       if self.joystick_state[1] in [max_forward_speed * speed_percentages[1]  , max_backward_speed * speed_percentages[1] ]:
          self.joystick_state[1] = self.joystick_state[1] * speed_percentages[2] / speed_percentages[1]
          print('vertical speed set to ' , self.joystick_state[1])

       if abs(self.joystick_state[0]) == max_sideways_speed * speed_percentages[1]:
          self.joystick_state[0] = self.joystick_state[0] * speed_percentages[2] / speed_percentages[1]
          print('horizontal speed set to ' , self.joystick_state[0])

    def on_R1_release(self):
       self.turbo = False    # release turbo
       
       if self.joystick_state[1] in [max_forward_speed , max_backward_speed]:
          self.joystick_state[1] = self.joystick_state[1] / speed_percentages[2] * speed_percentages[1]
          print('vertical speed set to ' , self.joystick_state[1])

       if abs(self.joystick_state[0]) == max_sideways_speed:
          self.joystick_state[0] = self.joystick_state[0] / speed_percentages[2] * speed_percentages[1]
          print('horizontal speed set to ' , self.joystick_state[0])

    # Right Trigger ______________________________________

    
    def on_R3_at_rest(self, button_id):
       if button_id == 3 and self.joystick_state[2] != 0:
          self.joystick_state[2]=  0
          # trigger new speed!
          print(' rotation set to zero ')


    def on_R3_up(self, value):
       # Perhaps we do an upwards rotation here
       pass


    def on_R3_down(self, value):
       # Perhaps we do an downwards rotation here
       pass

    def on_R3_left(self, value):
       speed = self.get_speed( abs(value) )  * max_rotation_speed * -1
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return

       if self.joystick_state[2] != speed:
          self.joystick_state[2] = speed
          print(' rotation speed set to '  , speed)
  
  
    def on_R3_right(self, value):
       
       speed = self.get_speed( abs(value) )  * max_rotation_speed 
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return

       if self.joystick_state[2] != speed:
          self.joystick_state[2] = speed
          print(' rotation speed set to '  , speed)


    # Left Trigger ______________________________________

    def on_L3_at_rest(self, button_id):
       if button_id == 0 and self.joystick_state[0] != 0:
          self.joystick_state[0]=  0
          # trigger new speed!
          print(' horizontal motion set to zero ')

       elif button_id ==1 and self.joystick_state[1] != 0:
          self.joystick_state[1] = 0
          # trigger new speed
          print('vertical  motion set to zero')

    
    def on_L3_up(self, value):
       
       speed = self.get_speed( abs(value) )  * max_forward_speed
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return


       if self.joystick_state[1] != speed:
          self.joystick_state[1] = speed
          print(' vertical speed set to '  , speed)



    def on_L3_down(self, value):
       
       speed = self.get_speed( abs(value) )  * max_backward_speed 
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return


       if self.joystick_state[1] != speed:
          self.joystick_state[1] = speed
          print(' vertical speed set to '  , speed)


    def on_L3_left(self, value):
       speed = self.get_speed( abs(value) )  * max_sideways_speed * -1
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return

       if self.joystick_state[0] != speed:
          self.joystick_state[0] = speed
          print(' horizontal speed set to '  , speed)

 
  
    def on_L3_right(self, value):
       speed = self.get_speed( abs(value) )  * max_sideways_speed 
       
#       if speed == 0:
          # reading was below minimum threshold, do nothing
#          return

       if self.joystick_state[0] != speed:
          self.joystick_state[0] = speed
          print(' horizontal speed set to '  , speed)
   

    # stand ______________

    def on_up_arrow_press(self):
        # Trigger stand up code
        print('stand up here')

    def on_down_arrow_press(self):
        # Trigger lie down code
        print('lie down here')




    # unused buttons set to pass ________

    def on_x_press(self):
       print('state is ' , self.joystick_state)

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

    def on_L2_press(self , value):
       pass 

    def on_L2_release(self):
       pass 

    def on_R2_press(self , value):
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
