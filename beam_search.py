"""
Connor Finn, Joshua Katz
4/28/2020

This Class will run the Beam Search algorithm. This is essentially // hill climber
"""

import numpy as np
from Robot import Robot
import os
import Simulator

class Beam_Search():


	def __init__(self, num_generations , population_size , simulator):
		self.num_generations = num_generations
		self.simulator = simulator
		self.population_size = population_size


	def make_population(self):
		# this method makes a population of robots
		# TO DO update the arguments for the robot object

		num = self.population_size
		pop = []
		current_dir = os.getcwd()
		urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
		for i in range(num):
			r = Robot(urdf, (0, 0, 0.4))
			r.randomize_genome()
			pop += [r]
		return pop

	def partition(self , A , B , p , r):
	    # rearange the subaray A[p . . r] in place.
	    # final result [vals < A[p] , A[p] ,  vals > A[p] ]
	    # The operations on array B will be identical
	    x = A[r]
	    i = p - 1
	    for j in range(p , r): 
	        if A[j] >= x:
	            i += 1
	            A[i] , A[j] = A[j] , A[i]
	            B[i] , B[j] = B[j] , B[i]

	    A[i +1] , A[r] = A[r] , A[i + 1]
	    B[i +1] , B[r] = B[r] , B[i + 1]
	    return(i + 1)

	def quick_sort(self , A , B ,  p , r ):
	# To sort an entire array A, the initial call is QUICKSORT(A, 0, length[A] - 1)
	# Array B is sorted in equivalence to array A
	    if p < r:
	        q = self.partition(A , B , p , r) 
	        self.quick_sort(A, B ,  p,  q - 1)
	        self.quick_sort(A, B , q + 1, r)
	        

	def run_algorithm(self):
		"""
		* this function performs a parametric optimization beam search algorithm on a single robot.
		* it will save the top performing robot, as well as learning curve 
		"""
		population_size = self.population_size
		simulator = self.simulator
		num_generations = self.num_generations
		current_dir = os.getcwd()
		urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
		simulated_robot = Robot(urdf, (0, 0, 0.4))
		simulated_robot.set_id(simulator.load_new_robot_urdf(simulated_robot))
		# make placeholders
		counter = 0		
		best_genome = None 			
		best_fit = 0 	
		evals = population_size  * ( num_generations + 1 )
		beam_fit = np.zeros(evals)			
		current_population = self.make_population()		
		current_population_fitness = [0] * self.population_size				
		#print("build robots")
		for k in range(self.population_size):
		#	print("initial robot  " , k)
			robot = current_population[k]					
			simulator.load_robot_parameters(robot.parameters, 0)
			robot.set_fitness(simulator.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness
			fitness = robot.get_fitness()				
			current_population_fitness[k] = fitness
			counter += 1

	#	print("origional robots evaluated, their fitness is  " , )
		for i in range(num_generations):		    	# perform mutations equal to num_Climb
#			print("start of gen , current population_fitness" , current_population_fitness)
			population = current_population.copy()
			population_fitness = current_population_fitness.copy()
#			print('gen' , i)
			for j in range(self.population_size):
				robot = population[j]						
				mut_loc , old_val = robot.mutate_genome()	# Mutation:  Keep track of mut location and previous vals
				simulator.load_robot_parameters(robot.parameters, 0)
				robot.set_fitness(simulator.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness
				fit_new = robot.get_fitness()
				population_fitness[j] = fit_new
				# BIG POINT - here we keep regardless if the change is better or not			
				if fit_new > best_fit:			# update learning curve
					best_fit = fit_new
					best_genome = robot.genome.copy()
				beam_fit[counter] = best_fit
				counter += 1
#			print(" ... ")
#			print("end of gen , current population_fitness" , current_population_fitness)
			# concat the populations and population fitnesses
			total_population = current_population + population
			total_population_fitness = current_population_fitness + population_fitness
			#print("before quick sort " , total_population_fitness)
			#print(" ... ")
			# sort the lists
			self.quick_sort(total_population_fitness , total_population , 0 , len(total_population) - 1)
			#print(" after quick sort " , total_population_fitness)
			#print(" ... ")
			# keep the top half
			current_population = total_population[:self.population_size]
			current_population_fitness = total_population_fitness[:self.population_size]
			#print("keep ", current_population_fitness)
#		print(counter)
		
		if not os.path.exists('./data'):
			os.mkdir('./data')

		np.savetxt("data/beam_genome.csv", best_genome, delimiter=",")
		np.savetxt("data/beam_learning.csv", beam_fit, delimiter=",")


# class Simulator():
#     # Place holder class: it returns a random fitness
#     # TO DO - import the actual simulator class
#     def calc_fitness(self, robot):
#         fit = np.random.uniform(0 , 10000)
#         robot.fitness = fit

if __name__ == '__main__':
    s = Simulator.Simulator(False)

    alg = Beam_Search(num_generations =  9, population_size = 10 , simulator = s)
    alg.run_algorithm()

