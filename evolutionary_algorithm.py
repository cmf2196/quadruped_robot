"""
Connor Finn, Joshua Katz
4/28/2020

This Class will run the EA algorithm. 
Bread and butter ea. Take a population, do crossover, mutate all robots a number of times, use truncation
selection.
"""

import numpy as np
from Robot import Robot
import os
import Simulator

class evolutionary_algorithm():

	def __init__(self , population_size  , simulator ):
		self.population_size = population_size
		self.simulator = simulator



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
	        


	def make_population(self , size ):
		# this method makes a population of robots
		# TO DO update the arguments for the robot object

		num = size
		pop = []
		current_dir = os.getcwd()
		urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
		for i in range(num):
			r = Robot(urdf, (0, 0, 0.4))
			r.randomize_genome()
			pop += [r]
		return pop

	def do_crossover(self , robot1 , robot2):
		"""
		This function performs a crossover between two robot genomes It will do so in a random direction
		and make two random cuts.
		Args:
			genome1, genome2: 2 numpy arrays of identical shape. These are the specific robot genomes
		Output:
			a new genome, which will be used for the child robot
		"""	

		genome1 = robot1.genome
		genome2 = robot2.genome
		size = len(genome1)					
		cut1 = np.random.randint(size)				# get both cuts, make sure they are different
		cut2 = np.random.randint(size)				#           "  "
		while cut1 == cut2:	
			cut1 = np.random.randint(size)
	
		first = min(cut1 , cut2)							# get the smaller and larget cuts
		second = max(cut1 , cut2)
		new_genome = genome1[ :first ] + genome2[first : second ] + genome1[ second:]	# make new genome
		
		# build child robot here!
		new_robot = Robot("path_to_urdf")
		new_robot.genome = new_genome
		new_robot.compute_parameters_from_genome()
		return new_robot					# return the child robot




	def run_algorithm(self , num_mutations , num_generations ):
		"""
		This method will run the evolutionary algorithm itself
		Args:
			robot: This is the robot object to be used in the simulation - could be taken in as an arg for the class instead
		Output:
			This outputs two csv files. 
			1. "ea_robot_genome.csv" represents the best performing genome
			2. "ea_learning.csv" is data for the algorithm's learning curve

		"""

		# ______  Step1: Population generation _________

		if not os.path.exists('./data'):
			os.mkdir('./data')

		population_size = self.population_size
		sim = self.simulator
		parent_size = int(population_size / 2 ) 
		evals = parent_size * (num_generations + 1) + population_size * num_mutations * num_generations
		parents = self.make_population(parent_size)		# make the population of parents 
		best_genome = None
		overall_fitness = np.zeros(evals )		# learning curve array
		ticker = 0									# keeps track of which robot eval we are on
		population_fitness = [0] * population_size 		# list used to keep track of the fitness of each robot in the population
		current_dir = os.getcwd()
		urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
		simulated_robot = Robot(urdf, (0, 0, 0.4))
		simulated_robot.set_id(sim.load_new_robot_urdf(simulated_robot))
		# ______  Step2: Crossover _________

		population = parents.copy() 							# make a shallow copy of the parents array
		for i in range(parent_size):							# select 2 random parents
			p1 = np.random.randint(population_size/2)			#		....
			p2 = np.random.randint(population_size/2)			#		....
											
			while p2 == p1:										# make sure they are not the same robot					
				p2 = np.random.randint(population_size/2)		# note: if pop size = 2; stuck here forever
			
			pr1 = parents[p1]									# perform a 2 cut crossover
			pr2 = parents[p2]									# 		....
			c = self.do_crossover(pr1 , pr2)							# 		....
			population = population + [c]						# add the child robot genomes to the population
		
		for j in range(parent_size):				# update the population fitness array 
			robot = population[j]														# select the robot
			sim.load_robot_parameters(robot.parameters, 0)
			robot.set_fitness(sim.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness			
			population_fitness[j] = robot.get_fitness()		

			if ticker == 0:
				overall_fitness[ticker] = population_fitness[j]	
			else;	
		
				if overall_fitness[ticker - 1] < population_fitness[j]:		# if the best overall robot thus far
					best_genome = robot.genome.copy()					# update the best robot's genome
					overall_fitness[ticker] = population_fitness[j]	
				else:
					overall_fitness[ticker] = overall_fitness[ticker - 1]


			ticker +=1	


		for m in range(num_generations):			# perform the EA
#			print("fitness at start of generation " , population_fitness)
			for z in range(parent_size , parent_size * 2):				# update the population fitness array 
				robot = population[z]														# select the robot
				sim.load_robot_parameters(robot.parameters, 0)
				robot.set_fitness(sim.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness			
				population_fitness[z] = robot.get_fitness()

				if overall_fitness[ticker - 1] < population_fitness[z]:		# if the best overall robot thus far
					overall_fitness[ticker] = population_fitness[z]			# add this as next value for the learning curve
					best_genome = robot.genome.copy()					# update the best robot's genome
				else:														# if lower, push the previous best score forward in learning curve
					overall_fitness[ticker] = overall_fitness[ticker - 1]
				ticker +=1	

	#		print(overall_fitness)
	#		print("fitness pre mutation " , population_fitness)
	#		print('generation: ' , m)
			# ______  Step3: Mutation _________

			for k in range(num_mutations):
	#			print('mut' , j)
				for p in range(population_size):					# mutate each robot in the population
					robot = population[p]							#			....
					
					mut_loc , old_val = robot.mutate_genome()	# Mutation:  Keep track of mut location and previous vals
					sim.load_robot_parameters(robot.parameters, 0)
					robot.set_fitness(sim.compute_walk_fitness(1000)[0])  # evaluate the robot's fitness
					fit_new = robot.get_fitness()		
							
					if fit_new > population_fitness[p]:				# if higher fitness then this robot's previous best : update
						population_fitness[p] = fit_new				# 			....
						
					else:
						robot.genome[mut_loc[0]][mut_loc[1]] = old_val	# if lower, revert the genome to it's previous state
					
					if overall_fitness[ticker - 1] < population_fitness[p]:		# if the best overall robot thus far
						overall_fitness[ticker] = population_fitness[p]			# add this as next value for the learning curve
						best_genome = robot.genome.copy()							# update the best robot's genome
					else:														# if lower, push the previous best score forward in learning curve
						overall_fitness[ticker] = overall_fitness[ticker - 1]
					ticker +=1													# end of mutation, add one to robot evaluation ticker

	# __________ Truncation Selection ____________
			self.quick_sort( population_fitness , population  , 0 , len(population_fitness) - 1)	# sort the population according to fitness rank ( sorts both population and population_fintess in tandem)

			parents = population[: parent_size] 				# perform truncation selection (choose top half)
			second_half = [0] * parent_size						# 	array of zeros
			population_fitness = population_fitness[: parent_size] + second_half		# keep first half (second half are zeros)

			if not os.path.exists('./data/generations'):
				os.mkdir('./data/generations')
			csv_name = "data/generations/ea_genome_gen_{}".format(m+1)
			np.savetxt(csv_name, best_genome, delimiter=",")
	# __________ Complete algorithm ______________


		np.savetxt("data/ea_genome.csv", best_genome, delimiter=",")
		np.savetxt("data/ea_learning.csv", overall_fitness, delimiter=",")


# class Simulator():
#     # Place holder class: it returns a random fitness
#     # TO DO - import the actual simulator class
#     def calc_fitness(self, robot):
#         fit = np.random.uniform(0 , 10000)
#         robot.fitness = fit

if __name__ == '__main__':
	s = Simulator.Simulator(False)
	# EA = evolutionary_algorithm(population_size = 60 , simulator=s)
	# EA.run_algorithm(num_mutations = 55 , num_generations = 9)
	EA = evolutionary_algorithm(population_size = 80 , simulator=s)
	EA.run_algorithm(num_mutations = 59 , num_generations = 21)


