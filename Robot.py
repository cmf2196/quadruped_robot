'''
Joshua Katz
4/27/2020

This is a generic robot class that has a urdf, a set of parameters for control,
and methods to change those fields
'''

import numpy as np 

class Robot:
    def __init__(self, urdf_path, parameters=() , genome = [] , num_joints = 12):
        self.urdf_path = urdf_path
        self.parameters = parameters
        self.genome = genome
        self.num_joints = num_joints
        self.fitness = 0  # default value, determined in simulation later

    def get_urdf_path(self):
        return self.urdf_path

    def set_urdf_path(self, path):
        self.urdf_path = path

    def get_parameters(self):
        return self.parameters

    def set_parameters(self, parameters):
        self.parameters = parameters

    def mutate_genome(self):
        # this method makes a small random change to the genme (a list of lists)
        # it will return the location of the mutation and the old value of the genome
        
        # pick location
        index_1 = np.random.randint(len(self.genome))        
        index_2 = np.random.randint(len(self.genome[0]))

        # make mutation
        old_val = self.genome[index_1][index_2]      
        loc = [index_1 , index_2]     
        mutation = np.random.uniform(0.9 , 1.1)
        self.genome[index_1][index_2] = old_val * mutation          # update the genome

        return loc , old_val      # important for negating a mutation


    def randomize_genome(self , symmetric = False):
        # creates a random list of lists as the robot's genome 
        # this uses some predefined constraints (hardcoded below for now)
        # the boolean value 'symmetric' is used to determine if we build half or all of the genome

        # for now the starting position and joit limit are hardcoded: (This could be read in later )  
        starting_pos = [0 , -1 * np.pi / (4.0) ,  -1 * np.pi / (2.0) , 0 , np.pi / (4.0), np.pi / (2.0), 0 ,  -1 * np.pi / (4.0),
         -1 * np.pi / (2.0), 0 , np.pi / (4.0), np.pi / (2.0) ]
        
        limit_1 = (np.radians(-135) , np.radians(135) )      # Joint limits for first shoulder motor
        limit_2 = (np.radians(-90) , np.radians(90) )        # joint limits for second shoulder motor
        limit_3 = (np.radians(-135) , np.radians(135) )      # joint limits for the elbow motor

        g = []                     
        
        if symmetric == False: 
            for i in range(self.num_joints):        # build the genome
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

                    c = np.random.uniform(lower , upper)  # get a c value within the joint limits
                    a = min(c - lower  , upper - c)       # get an A value according to the closets joint limit

                    if starting_pos[i] <= c + a and starting_pos[i] >= c - a : 
                        g += [[a , c]]                    # if the joint starting position is within the possible values
                        viable = True      
        
        else: 
            half = self.num_joints / 2   # assuming symmetry 
            for i in range(half):        # build the genome
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

                    c = np.random.uniform(lower , upper)    # get a c value within the joint limits
                    a = min(c - lower  , upper - c)         # get an A value according to the closets joint limit

                    if starting_pos[i] <= c + a and starting_pos[i] >= c - a : 
                        g += [[a , c]]                      # if the joint starting position is within the possible values
                        viable = True  

        self.genome = g


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

def main():

    r = Robot('path')
    r.randomize_genome()
    print(r.genome)
    print(r.mutate_genome())
    print(r.genome )

main()

