'''
Author: Connor Finn
Date: September 4, 2020

Details:
   This file controlls the ps4 controller. It uses a module, which was downloaded and slightly edited for 
   our use case. The origional documentation is here, https://pypi.org/project/pyPS4Controller/

   To set up the ps4 controller, first connect it to your raspberry pi using the GUI. Instructions are 
   provided clearly here https://pimylifeup.com/raspberry-pi-bluetooth/ 

   The current motion settings are listed below:
      Left Joystick - forward and sideways motion
      Right Joystick - Rotation (only turning at the moment, no up and down)
      Up arrow - Stand up
      Down arrow - lay down
      R1 - Turbo
      x - currently prints state (this is for debugging)


    Speed Determination
       Currently speed is given by discrete values. There are predefined maximum speeds for each direciton.
       Then, depending on where the joystick is positioned relative to the predefined thresholds, a percentage of 
       the maximum speed is set to the  joystick_state, defined as [horizontal  motion, vertical motion,  rotation ].
       Forward and sideways motion have three speeds, the top speed being achieved using the Turbo button
       Rotation has two speeds, low and medium (need to consider this  when defining the top rotational speed) because
       
     Turbo Button
         the turbo button has no impact on rotational speed.
         The turbo button always increases speed by one, this is to prevent a jump from walk to run
             walk + turbo = trot
             trot + turbo = run

          A skip from walk to trot DOES OCCUR
          A skip from trot to still DOES OCUR
       
     At the end of each method, we will encorperate the actual  call to our robot. 
'''

import threading
import time
from pyPS4Controller_edit.controller2 import Controller, Event



class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.joystick_state = [0, 0, 0]  # (sideways , forward , rotation)
        self.turbo = False  # This will be R1 -> used to get run speed

        self.max_sideways_speed = 0.5  # m/s
        self.max_forward_speed = 0.5  # m/s
        self.max_backward_speed = -0.5  # m/s
        self.max_rotation_speed = 1  # rad/s

        self.speed_percentages = [0.2, 0.6,
                                  1]  # walk, trot , run  note, rotation just used the second two!

        self.max_threshold = 20000     # equal or greater for med / high speed
        self.min_threshold = 10000      # equal or greater for low speed


        self.R3 = False

        self.up_arrow = False
        self.down_arrow = False
        self.L1 = False

    def initialize_connection(self):
        t = threading.Thread(target=self.listen)
        t.start()

    def get_state(self):
        return self.joystick_state

    def get_speed(self, val):
        # Method which indicates the speed percentage to use

        if val >= self.min_threshold and val < self.max_threshold and self.turbo == False:  # low speed
            return self.speed_percentages[0]

        if val >= self.min_threshold and val < self.max_threshold and self.turbo == True:  # low speed
            return self.speed_percentages[1]

        elif val >= self.max_threshold and self.turbo == False:  # medium speed
            return self.speed_percentages[1]

        elif val >= self.max_threshold and self.turbo == True:  # meax speed
            return self.speed_percentages[2]

        else:  # signal too small
            return 0

    def get_no_turbo_speed(self, val):
        # Method which indicates the speed percentage to use

        if val >= self.min_threshold and val < self.max_threshold:  # medium speed
            return self.speed_percentages[1]

        elif val >= self.max_threshold:  # meax speed
            return self.speed_percentages[2]

        else:  # signal too small
            return 0

    def on_R1_press(self):
        # Turbo Button is on

        self.turbo = True  # set turbo

        # trot to run
        if self.joystick_state[1] in [
            self.max_forward_speed * self.speed_percentages[1],
            self.max_backward_speed * self.speed_percentages[1]]:
            self.joystick_state[1] = self.joystick_state[1] * \
                                     self.speed_percentages[2] / \
                                     self.speed_percentages[1]
            print('vertical speed set to ', self.joystick_state[1])

        if abs(self.joystick_state[0]) == self.max_sideways_speed * \
                self.speed_percentages[1]:
            self.joystick_state[0] = self.joystick_state[0] * \
                                     self.speed_percentages[2] / \
                                     self.speed_percentages[1]
            print('horizontal speed set to ', self.joystick_state[0])

        # walk to trot
        if self.joystick_state[1] in [
            self.max_forward_speed * self.speed_percentages[0],
            self.max_backward_speed * self.speed_percentages[0]]:
            self.joystick_state[1] = self.joystick_state[1] * \
                                     self.speed_percentages[1] / \
                                     self.speed_percentages[0]
            print('vertical speed set to ', self.joystick_state[1])

        if abs(self.joystick_state[0]) == self.max_sideways_speed * \
                self.speed_percentages[0]:
            self.joystick_state[0] = self.joystick_state[0] * \
                                     self.speed_percentages[1] / \
                                     self.speed_percentages[0]
            print('horizontal speed set to ', self.joystick_state[0])

    def on_R1_release(self):
        # Turbo button is off

        self.turbo = False  # release turbo

        # run to trot
        if self.joystick_state[1] in [self.max_forward_speed,
                                      self.max_backward_speed]:
            self.joystick_state[1] = self.joystick_state[1] / \
                                     self.speed_percentages[2] * \
                                     self.speed_percentages[1]
            print('vertical speed set to ', self.joystick_state[1])

        if abs(self.joystick_state[0]) == self.max_sideways_speed:
            self.joystick_state[0] = self.joystick_state[0] / \
                                     self.speed_percentages[2] * \
                                     self.speed_percentages[1]
            print('horizontal speed set to ', self.joystick_state[0])

        # trot to walk
        if self.joystick_state[1] in [
            self.max_forward_speed * self.speed_percentages[1],
            self.max_backward_speed * self.speed_percentages[1]]:
            self.joystick_state[1] = self.joystick_state[1] / \
                                     self.speed_percentages[1] * \
                                     self.speed_percentages[0]
            print('vertical speed set to ', self.joystick_state[1])

        if abs(self.joystick_state[0]) == self.max_sideways_speed * \
                self.speed_percentages[1]:
            self.joystick_state[0] = self.joystick_state[0] / \
                                     self.speed_percentages[1] * \
                                     self.speed_percentages[0]
            print('horizontal speed set to ', self.joystick_state[0])

    # Right Trigger ______________________________________

    def on_R3_at_rest(self, button_id):
        # This is when the Right Joystick registers a value of zero
        # button id is used to determine which axis has crossed the zero plane

        if button_id == 3 and self.joystick_state[
            2] != 0:  # here we are only considering sideways rotation
            self.joystick_state[2] = 0
            print(' Rotation set to zero')

    def on_R3_up(self, value):
        # Perhaps we do an upwards rotation here
        pass

    def on_R3_down(self, value):
        # Perhaps we do an downwards rotation here
        pass

    def on_R3_left(self, value):
        # Joystick is pressed left

        speed = self.get_no_turbo_speed(
            abs(value)) * self.max_rotation_speed * -1

        if self.joystick_state[2] != speed:
            self.joystick_state[2] = speed
            print(' rotation speed set to ', speed)

    def on_R3_right(self, value):

        speed = self.get_no_turbo_speed(abs(value)) * self.max_rotation_speed

        if self.joystick_state[2] != speed:
            self.joystick_state[2] = speed
            print(' rotation speed set to ', speed)

    # Left Trigger ______________________________________

    def on_L3_at_rest(self, button_id):
        if button_id == 0 and self.joystick_state[0] != 0:
            self.joystick_state[0] = 0
            # trigger new speed!
            print(' horizontal motion set to zero ')

        elif button_id == 1 and self.joystick_state[1] != 0:
            self.joystick_state[1] = 0
            # trigger new speed
            print('vertical  motion set to zero')

    def on_L3_up(self, value):

        speed = self.get_speed(abs(value)) * self.max_forward_speed

        if self.joystick_state[1] != speed:
            self.joystick_state[1] = speed
            print(' vertical speed set to ', speed)

    def on_L3_down(self, value):

        speed = self.get_speed(abs(value)) * self.max_backward_speed

        if self.joystick_state[1] != speed:
            self.joystick_state[1] = speed
            print(' vertical speed set to ', speed)

    def on_L3_left(self, value):
        speed = self.get_speed(abs(value)) * self.max_sideways_speed * -1

        if self.joystick_state[0] != speed:
            self.joystick_state[0] = speed
            print(' horizontal speed set to ', speed)

    def on_L3_right(self, value):
        speed = self.get_speed(abs(value)) * self.max_sideways_speed

        if self.joystick_state[0] != speed:
            self.joystick_state[0] = speed
            print(' horizontal speed set to ', speed)

    # stand ______________

    def on_up_arrow_press(self):
        # Trigger stand up code
        self.up_arrow = True

    def on_down_arrow_press(self):
        # Trigger lie down code
        self.down_arrow = True

    # unused buttons set to pass ________

    def on_x_press(self):
        print('state ', self.joystick_state)

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
        self.L1 = True

    def on_L1_release(self):
        self.L1 = False

    def on_L2_press(self, value):
        pass

    def on_L2_release(self):
        pass

    def on_R2_press(self, value):
        pass

    def on_R2_release(self):
        pass

    def on_up_down_arrow_release(self):
        self.up_arrow = False
        self.down_arrow = False

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
        self.R3 = not self.R3

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

    def R3_at_rest(self):
        return self.button_id in [3,
                                  4] and self.button_type == 2 and self.value == 0

    def L3_at_rest(self):
        return self.button_id in [0,
                                  1] and self.button_type == 2 and self.value == 0


if __name__ == "__main__":
    controller = MyController(interface="/dev/input/js0",
                              connecting_using_ds4drv=False,
                              event_definition=MyEventDefinition)
    controller.initialize_connection()
    # controller.debug = True  # you will see raw data stream for any button press, even if that button is not mapped
    # you can start listening before controller is paired, as long as you pair it within the timeout window
    # t = threading.Thread(target=controller.listen, daemon=True)
    # controller.listen(timeout=60)
    # t.start()
    # t.join()
    while (1):
        control = controller.get_state()
        print(control)
        time.sleep(0.01)
