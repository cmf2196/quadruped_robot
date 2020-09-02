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
from pyPS4Controller.controller import Controller, Event


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)	

	# We will define our own states - this solves the joystick issue
        self.R3 = [0 , 0]   # (vertical , horizontal)
        self.L3 = [0 , 0]   # (Vertical , horizontal)
        self.x  = 0
        self.triangle  = 0
        self.circle = 0
        self.square = 0
        self.left_arrow = 0
        self.right_arrow = 0 
        self.down_arrow = 0
        self.up_arrow = 0
        # for our purposes this is plenty of buttons. We can always add more if we need them

    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")


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

    def R3_at_rest(self):
        return self.button_id == 4 and self.button_type == 2 and self.value == 0 


controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False, event_definition=MyEventDefinition)
controller.debug = True  # you will see raw data stream for any button press, even if that button is not mapped
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
