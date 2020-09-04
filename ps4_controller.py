
'''
This file will enable use for the ps4 controller. 
to connect the controller to the pi, follow this tutorial
https://pimylifeup.com/raspberry-pi-bluetooth/ 

This script uses the module given here https://pypi.org/project/pyPS4Controller/.

The  library is small, has functions like x pressed and x released. 
it seems to have issues with the joysticks however, if R3 is up and to the left, then travels to up 
and to the right, the module reads at rest at the point where R3 passes from left to right. 

It seems like a lapse in judgement by the developers. unless this is standard. 

we can switch to pygame, however this is a large download.  we can discuss thursday




'''


from pyPS4Controller_edit.controller2 import Controller, Event

#from config import gait_speeds      # [walk , trot , run] 

gait_speed = [1 , 2 , 3] 



class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)	

	# We will define our own states - this solves the joystick issue
        self.joystick_state = [0 , 0 , 0]   # (sideways , forward , rotation)
        self.turbo = False  # This will be R1 -> used to get run speed

    def on_x_press(self):
        pass

    def on_circle_press(self):
       pass

    def on_x_release(self):
        pass
   
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



    def on_R3_at_rest(self, button_id):
       print("resting, id = " , button_id  )
       self.joystick_state[:1]=  [0 , 0]

    def on_R3_up(self, value):
       print(value)

    def on_R3_down(self, value):
       print(value)
    def on_R3_left(self, value):
       print(value)

    def on_R3_right(self, value):
       print(value)

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
       if abs(value)  >= 20000 and self.turbo == True and self.joystick_state[1] != gait_speed[2]:    # run
          self.joystick_state[1] = gait_speed[2]
          # Trigger new speed
          print(' vertical speed set to run')
       elif  abs(value) < 20000 and self.joystick_state[1] != gait_speed[0]:    # walk 
          self.joystick_state[1] = gait_speed[0]
          # Trigger new speed
          print(' vertical speed set to walk')

       elif abs(value) >= 20000 and self.turbo == False and self.joystick_state[1] != gait_speed[1]:  # trot
          self.joystick_state[1] = gait_speed[1]
          # Trigger new Speed
          print(' vertical speed set to trot')

    def on_L3_down(self, value):

       if abs(value)  >= 20000 and self.turbo == True and self.joystick_state[1] != -1 * gait_speed[2]:    # run
          self.joystick_state[1] = -1* gait_speed[2]
          # Trigger new speed
          print(' vertical speed set to run')
       elif  abs(value) < 20000 and self.joystick_state[1] != -1 * gait_speed[0]:    # walk 
          self.joystick_state[1] = -1 *gait_speed[0]
          # Trigger new speed
          print(' vertical speed set to walk')

       elif abs(value) >= 20000 and self.turbo == False and self.joystick_state[1] != gait_speed[1]:  # trot
          self.joystick_state[1] = gait_speed[1]
          # Trigger new Speed
          print(' vertical speed set to trot')

    def on_L3_left(self, value):
       print(value)

    def on_L3_right(self, value):
       print(value)

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
