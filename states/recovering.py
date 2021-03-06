from state import State
import time



class Recovering(State):

    def __init__(self):
        self.clock_max = 0
        self.clock = 0

    def enter(self, robot):
        pass


    def exit(self, robot , new_state):
        # new_state is a string (the name of the new state to be entered)
        self.clock = 0
        robot.imu_state = ['Stable' , 'Stable']
        return new_state

    def update(self, robot, controller_state):
        # This needs to be updated
        if robot.motors:
            time.sleep(1)
            robot.motor_controller.recover_from_fall()
        return self.exit(robot , 'Lay')

