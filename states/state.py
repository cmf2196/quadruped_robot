# Connor Finn
# September 18, 2020
# I am following this tutorial https://dev.to/karn/building-a-simple-state-machine-in-python

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self) ) 

    

    def update(self, robot ,  controller_state):
        """
        Handle events that are delegated to this State.
        """
        pass

    def exit(self , new_state):
        # This will tell the state machine what state to transition into
        return 

    def enter(self , robot):
        # this will probably be usefule. not sure yet
        pass


    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__