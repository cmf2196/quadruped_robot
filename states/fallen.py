from state import State


class Fallen(State):


    def enter(self, robot):
        pass


    def exit(self, new_state):
        # new_state is a string (the name of the new state to be entered)
        return new_state

    def update(self, robot, controller_state):
        # if x is pressed
        if controller_state[7] == 1:
            return self.exit('Recovering')

        else:
            # always transition to recovering from falling immediately for now
            return self.exit('Recovering')
          