import numpy as np 

class random_search():
    def __init__(self, num_iterations , robot):
        self.num_iterations = num_iterations
        self.robot = robot


    def run_algorithm(self ):

        """ 
        * this function will perform a random search for the best robot
        * this is done by generating a new random robot, comparing it to the current best, 
        * and keeping it only if it achieves higher fitness
        """     
        robot = self.robot
        num_search = self.num_iterations

        rs_fit = np.zeros(num_search)               # learning curve array
        best_fit = 0                                # best fitness place holder
        best_array = np.empty(8)                    # top performing genome place holder
        
        for i in range(num_search):                 # do the random search  
            print('counter: ', i)

            robot.build_genome()                    # build the robot's genome
            fitness = robot.calc_fitness()          # evaluate the robot's fitness

            if fitness > best_fit:                  # if the fitness is better than current best
                best_fit = fitness                  # update placeholders
                best_array = np.copy(robot.genome)  #       ....

            rs_fit[i] = best_fit                    # add the best fit to the learning curve

        np.savetxt("rs_robot.csv", best_array, delimiter=",")
        np.savetxt("rs_learning.csv", rs_fit, delimiter=",")


