"""
Connor Finn
4/28/2020

This Class will run the hill climber algorithm. It requires the related robot and simulator classes
"""

import numpy as np 
from Robot import Robot
# TO DO - import the  simulator class

class Hill_Climber():

	def __init__(self, num_iterations , robot , simulator):
		self.num_iterations = num_iterations
		self.robot = robot
		self.simulator = simulator


	def run_algorithm(self):

		"""
		* this function performs a parametric optimization hill climber algorithm on a single robot.
		* it will save the top performing robot, as well as learning curve 

		"""
		robot = self.robot 
		simulator = self.simulator
		num_climb = self.num_iterations

		hc_fit = np.zeros(num_climb)				# create an array to store the learning curve data
		robot.randomize_genome()					# build the genome
		simulator.calc_fitness(robot)				# get the fitness of the first robot
		best_fit = robot.get_fitness()				
		best_genome = robot.genome 					# initialize the best array to keep track
		hc_fit[0] = best_fit						# place it as the first value in the learning curve array

		for i in range(1 ,num_climb):		    	# perform mutations equal to num_Climb
			print('mut' , i)						
			mut_loc , old_val = robot.mutate_genome()	# Mutation:  Keep track of mut location and previous vals
			simulator.calc_fitness(robot)				# get the fitness of the first robot
			fit_new = robot.get_fitness()	
			
			if fit_new > best_fit:					# if better update the top fitness, and top performing genome
				best_fit = fit_new					
				best_genome = robot.genome.copy()

			else:									# if worse, revert the genome back to its previous state. 
				robot.genome[mut_loc[0]][mut_loc[1]] = old_val		# only need to adjust genome
			hc_fit[i] = best_fit									# robot is rebuilt prior to fitness calculation each time
			
		np.savetxt("hc_genome.csv", best_genome, delimiter=",")
		np.savetxt("hc_learning.csv", hc_fit, delimiter=",")


class Simulator():
    # Place holder class: it returns a random fitness
    # TO DO - import the actual simulator class
    def calc_fitness(self, robot):
        fit = np.random.uniform(0 , 10000)
        robot.fitness = fit

if __name__ == '__main__':
    s = Simulator()
    r = Robot('path_to_urdf' )

    alg = Hill_Climber(100 , r , s)
    alg.run_algorithm()

