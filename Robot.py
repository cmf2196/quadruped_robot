'''
Joshua Katz
4/27/2020

This is a generic robot class that has a urdf, a set of parameters for control,
and methods to change those fields
'''

import numpy as np
import pybullet as p
import math


class Robot:
    stand_position = [0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0), 0,
                      -1 * np.pi / (4.0),
                      -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0)]

    def __init__(self, urdf_path, start_pos=(0, 0, 1), start_rpy=(0, 0, 0), parameters=(), genome=(), num_joints=12):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_or = p.getQuaternionFromEuler(start_rpy)
        self.parameters = parameters
        self.genome = genome
        self.num_joints = num_joints
        self.fitness = 0  # default value, determined in simulation later
        self.id = None  # if robot is loaded into pybullet simulation, this will be updated
        self.mode = "idle"  # possible modes: crouch, walk, jump, turn right, etc
        self.target_position = Robot.stand_position  # array of n_joint floats

        # temporary init feature, create valid parameters
        self.compute_temporary_walking_parameters()

    def get_urdf_path(self):
        return self.urdf_path

    def set_urdf_path(self, path):
        self.urdf_path = path

    def get_start_pos(self):
        return self.start_pos

    def set_start_pos(self, s):
        self.start_pos = s

    def get_start_or(self):
        return self.start_or

    def set_start_or_from_euler(self, euler):
        self.start_or = p.getQuaternionFromEuler(euler)

    def get_parameters(self):
        return self.parameters

    def set_parameters(self, parameters):
        self.parameters = parameters

    def compute_temporary_walking_parameters(self):
        # Parameters for now are filled in by hand. Begin by creating array of 0's
        parameters = []
        for x in range(0, 12):
            parameters.append([0, 0, 0])  # [A,B,C]

        # Next I will adjust A for each group. I assume that all motors of same type have same amplitude. Therefore,
        # there are three amplitudes:

        A_1 = 0
        A_2 = np.pi / 4.0  # just a guess, 90 deg is not a bad place to start
        A_3 = 0  # np.pi / 4.0

        # Next we need B values. I assume half will be 0 and half will be ~pi.

        # Finally we need C values. For now I will assume these are all 0.

        # So now let's modify the paramater list:

        # Amplitudes:
        for x in range(0, 12, 3):  # modifying every third motor of type 1
            parameters[x][0] = A_1
        for x in range(1, 12, 3):  # modifying every third motor of type 2
            parameters[x][0] = A_2
            # if x == 1 or x == 4:
            #    parameters[x][1] = np.pi
        for x in range(2, 12, 3):  # modifying every third motor of type 3
            parameters[x][0] = A_3

        # change c values to maintain initial posture

        # showing parameters for debugging purposes
        # print(parameters)

        self.parameters = parameters

    def compute_parameters_from_genome(self):
        # uses genome to compute b values and inserts them into new array
        parameters = []

        for i in range(0, len(self.genome)):
            A, C = self.get_genome()[i]
            N = Robot.stand_position[i]  # Using start position as reference position

            try:
                B1 = math.asin((N - C) / A)  # other value can be found mathamatically
            except ValueError:
                print("B could not be calculated")
                print("A is: " + str(A))
                print("C is: " + str(C))
                print("N is: " + str(N))
            # could pick the B randomly, but it is probably better to be deterministic...
            if B1 > 0:
                B2 = math.pi - B1
            if B1 < 0:
                B2 = 3 * math.pi - B1

            if np.random.randint(0, 2):
                B = B1
            else:
                B = B2

            parameters.append([A, B1, C])  # choosing B1, could use random B instead

        self.parameters = parameters

        # Equation: b = arcsin((N-C)/A)

        # process: loop through genome and insert b values using existing A and C values

        self.set_parameters(parameters)

    def get_genome(self):
        return self.genome

    def set_genome(self, g):
        self.genome = g

    def mutate_genome(self):
        # this method makes a small random change to the genme (a list of lists)
        # it will return the location of the mutation and the old value of the genome

        # pick location
        index_1 = np.random.randint(len(self.genome))
        index_2 = np.random.randint(len(self.genome[0]))

        # make mutation
        old_val = self.genome[index_1][index_2]
        loc = [index_1, index_2]
        mutation = np.random.uniform(0.9, 1.1)
        self.genome[index_1][index_2] = old_val * mutation  # update the genome

        return loc, old_val  # important for negating a mutation

    def randomize_genome(self, symmetric=False):
        # creates a random list of lists as the robot's genome 
        # this uses some predefined constraints (hardcoded below for now)
        # the boolean value 'symmetric' is used to determine if we build half or all of the genome

        # for now the starting position and joit limit are hardcoded: (This could be read in later )  
        starting_pos = Robot.stand_position
        '''[0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0), 0,
                        -1 * np.pi / (4.0),
                        -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0)]'''

        # limit_1 = (np.radians(-135), np.radians(135))  # Joint limits for first shoulder motor
        limit_1 = (np.radians(-10), np.radians(10))  # THIS IS A TEMPORARY LIMIT TO MAKE WALK EASIER
        limit_2 = (np.radians(-90), np.radians(90))  # joint limits for second shoulder motor
        limit_3 = (np.radians(-135), np.radians(135))  # joint limits for the elbow motor

        g = []

        if symmetric == False:
            for i in range(self.num_joints):  # build the genome
                viable = False
                while viable == False:
                    # set the correct joint limits
                    if i % 3 == 0:
                        lower = limit_1[0]
                        upper = limit_1[1]
                    elif i % 3 == 1:
                        lower = limit_2[0]
                        upper = limit_2[1]
                    else:
                        lower = limit_3[0]
                        upper = limit_3[1]

                    c = np.random.uniform(lower, upper)  # get a c value within the joint limits
                    a_max = min(c - lower, upper - c)  # get an A value according to the closets joint limit
                    a = np.random.uniform(0, a_max)

                    if starting_pos[i] <= c + a and starting_pos[i] >= c - a:
                        g += [[a, c]]  # if the joint starting position is within the possible values
                        viable = True

        else:
            half = int(self.num_joints / 2)  # assuming symmetry
            for i in range(half):  # build the genome
                viable = False
                while viable == False:
                    # set the correct joint limits
                    if i % 3 == 0:
                        lower = limit_1[0]
                        upper = limit_1[1]
                    elif i % 3 == 1:
                        lower = limit_2[0]
                        upper = limit_2[1]
                    else:
                        lower = limit_3[0]
                        upper = limit_3[1]

                    c = np.random.uniform(lower, upper)  # get a c value within the joint limits
                    a = min(c - lower, upper - c)  # get an A value according to the closets joint limit

                    if starting_pos[i] <= c + a and starting_pos[i] >= c - a:
                        g += [[a, c]]  # if the joint starting position is within the possible values
                        viable = True
                        g = g + g  # duplicated to copy genome to other half!!!

        self.genome = g
        self.compute_parameters_from_genome()

    def get_fitness(self):
        return self.fitness

    def set_fitness(self, fitness):
        self.fitness = fitness

    def compute_fitness(self):
        self.fitness = 0  # real computation from simulator goes here

    def get_id(self):
        return self.id

    def set_id(self, id):
        self.id = id

    def get_mode(self):
        return self.mode

    def set_mode(self, mode):
        self.mode = mode

        if mode == "hold":
            states = p.getJointStates(self.id, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            joint_pos = []
            for state in states:
                joint_pos.append(state[0])
            self.target_position = joint_pos

    def get_target_position(self):
        return self.target_position

    def set_target_position(self, pos):
        self.target_position = pos

    # computes the target position for the robot based on the mode
    def compute_target_position(self, t):
        if self.get_mode() == "idle":
            self.target_position = Robot.stand_position
        elif self.get_mode() == "walk":
            # calculation based on parameters and time
            target = []
            for motor_params in self.parameters:
                A = motor_params[0]
                B = motor_params[1]
                C = motor_params[2]
                W = 0.01  # hard coded, will become a parameter later
                motor_target = A * np.sin(t * W + B) + C
                target.append(motor_target)
            self.target_position = target

        elif self.get_mode() == "hold":
            pass  # if hold, do not change the target position!

        else:
            print("Mode not recognized")
            exit()

        return self.target_position

    # useful for copying a robot to make a child in evolution
    def copy_robot(self, robot):
        self.urdf_path = robot.urdf_path
        self.parameters = robot.parameters
        self.fitness = robot.fitness


def main():
    r = Robot('path')
    r.randomize_genome()
    print(r.genome)
    print(r.mutate_genome())
    print(r.genome)
    print(r.compute_parameters_from_genome())

# main()
