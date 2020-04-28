'''
Joshua Katz
4/27/2020

This is a generic robot class that has a urdf, a set of parameters for control,
and methods to change those fields
'''


class Robot:
    def __init__(self, urdf_path, parameters=()):
        self.urdf_path = urdf_path
        self.parameters = parameters
        self.fitness = 0  # default value, determined in simulation later

    def get_urdf_path(self):
        return self.urdf_path

    def set_urdf_path(self, path):
        self.urdf_path = path

    def get_parameters(self):
        return self.parameters

    def set_parameters(self, parameters):
        self.parameters = parameters

    def mutate_parameters(self, n):
        # Need a method to make a small change to the parameters
        # In this case, based on some arbitrary number n
        pass

    def randomize_parameters(self):
        # Use random number generator (with some constraints) to create
        # random parameters
        pass

    def get_fitness(self):
        return self.fitness

    def set_fitness(self, fitness):
        self.fitness = fitness

    def compute_fitness(self):
        self.fitness = 0  # real computation from simulator goes here

    # useful for copying a robot to make a child in evolution
    def copy_robot(self, robot):
        self.urdf_path = robot.urdf_path
        self.parameters = robot.parameters
        self.fitness = robot.fitness




