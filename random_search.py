"""
Connor Finn, Joshua Katz
4/29/2020

This file is used to run the random search algorithm on our robot within the bullet simulator
"""

import numpy as np
from Robot import Robot
import os
import Simulator


# TO DO - Need to import in the simulator class

class Random_Search():
    def __init__(self, num_iterations, robot, simulator):
        self.num_iterations = num_iterations
        self.robot = robot
        self.simulator = simulator

    def run_algorithm(self):
        # This will perform a random search algorithm
        # It will return two csv files:
        #   1. The optimal genome in rs_genome.csv
        #   2. The learning curve in rs_learning.csv

        robot = self.robot
        simulator = self.simulator
        num_search = self.num_iterations
        rs_fit = np.zeros(num_search)  # learning curve array
        best_fit = -10  # best fitness place holder
        best_genome = None  # top performing genome place holder

        for i in range(num_search):  # do the random search
            print('counter: ', i)
            robot.randomize_genome()  # build the robot's genome
            simulator.load_robot_parameters(robot.parameters, 0)
            robot.set_fitness(simulator.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness
            fitness = robot.get_fitness()

            if fitness > best_fit:  # if the fitness is better than current best
                best_fit = fitness  # update placeholders
                best_genome = robot.genome.copy()  # ....

            rs_fit[i] = best_fit  # add the best fit to the learning curve

        print("Genome: " + str(best_genome))
        print(best_fit)
        np.savetxt("rs_genome.csv", best_genome, delimiter=",")
        np.savetxt("rs_learning.csv", rs_fit, delimiter=",")


'''class Simulator():
    # Place holder class: it returns a random fitness
    # TO DO - import the actual simulator class
    def calc_fitness(self, robot):
        fit = np.random.uniform(0, 10000)
        robot.fitness = fit'''


if __name__ == '__main__':
    s = Simulator.Simulator(False)
    current_dir = os.getcwd()
    urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
    r = Robot(urdf, (0, 0, 0.4))
    r.set_id(s.load_new_robot_urdf(r))

    alg = Random_Search(100, r, s)
    alg.run_algorithm()
