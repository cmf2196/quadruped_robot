from state import State
import time



class Recovering(State):

    def __init__(self):
        self.clock_max = 0
        self.clock = 0

    def enter(self, robot):
        pass


    def exit(self, new_state):
        # new_state is a string (the name of the new state to be entered)
        self.clock = 0
        return new_state

    def update(self, robot, controller_state):
        # This needs to be updated
        time.sleep(1)
        robot.motor_controller.recover_from_fall()
        self.exit('Lay')

