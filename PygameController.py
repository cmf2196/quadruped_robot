"""
Joshua Katz
9/10/2020
"""

import time
import pygame


class PygameController:

    def __init__(self):
        self.joystick = None
        pygame.init()
        self.connect_to_controller()

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

    def get_multi_axis(self):
        self.refresh_controller()

        # 0&1 for left stick, 2&3 for right stick
        return [self.joystick.get_axis(0),
                -self.joystick.get_axis(1),
                self.joystick.get_axis(2),
                -self.joystick.get_axis(3)]

    def multi_axis_to_velocity(self):
        axis = self.get_multi_axis()
        mults = [0.25, 0.25, 1]

        if self.get_button(5):
            mults = [0.5, 0.5, 1]

        x_vel = axis[0]*mults[0]
        if abs(x_vel) < 0.1:
            x_vel = 0

        y_vel = axis[1]*mults[1]
        if abs(y_vel) < 0.1:
            y_vel = 0

        ang_vel = -axis[2]*mults[2]
        if abs(ang_vel) < 0.1:
            ang_vel = 0

        return x_vel, y_vel, ang_vel

    def get_button(self, button):
        self.refresh_controller()
        return self.joystick.get_button(button)

    @staticmethod
    def refresh_controller():
        pygame.event.get()


if __name__ == "__main__":

    controller = PygameController()
    try:
        while (1):
            print(controller.get_multi_axis())
            print(controller.get_button(0))

            time.sleep(0.5)
    except KeyboardInterrupt:
        exit()
