import numpy as np 

# TO DO - Need to import in the robot and simulator classes

class random_search():
    def __init__(self, num_iterations , robot , simulator):
        self.num_iterations = num_iterations
        self.robot = robot
        self.simulator = simulator


    def run_algorithm(self ):

        """ 
        * this function will perform a random search for the best robot
        * this is done by generating a new random robot, comparing it to the current best, 
        * and keeping it only if it achieves higher fitness
        """     

        robot = self.robot
        simulator = self.simulator
        num_search = self.num_iterations

        rs_fit = np.zeros(num_search)               # learning curve array
        best_fit = 0                                # best fitness place holder
        best_array = None                           # top performing genome place holder
        
        for i in range(num_search):                 # do the random search  
            print('counter: ', i)

            robot.randomize_genome()                # build the robot's genome
            simulator.calc_fitness()                # evaluate the robot's fitness
            fitness = robot.get_fitness()

            if fitness > best_fit:                  # if the fitness is better than current best
                best_fit = fitness                  # update placeholders
                best_array = robot.genome.copy()    #       ....

            rs_fit[i] = best_fit                    # add the best fit to the learning curve

        np.savetxt("rs_genome.csv", best_array, delimiter=",")
        np.savetxt("rs_learning.csv", rs_fit, delimiter=",")


