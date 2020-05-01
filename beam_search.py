"""
Connor Finn
4/28/2020

This Class will run the Beam Search algorithm. This is essentially // hill climber
"""
import numpy as np 
from Robot import Robot
# TO DO - import the  simulator class

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
		for i in range(num):
			r = Robot("path_to_urdf")
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
	        if A[j] <= x:
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

		# make placeholders
		counter = 0		
		best_genome = None 			
		best_fit = 0 	
		beam_fit = np.zeros(num_generations * self.population_size)			
		current_population = self.make_population()		
		current_population_fitness = [0] * self.population_size				
				

		for i in range(num_generations):		    	# perform mutations equal to num_Climb
			population = current_population.copy()
			population_fitness = current_population_fitness.copy()
			print('gen' , i)
			for j in range(self.population_size):
				robot = population[j]						
				mut_loc , old_val = robot.mutate_genome()	# Mutation:  Keep track of mut location and previous vals
				simulator.calc_fitness(robot)				# get the fitness of the first robot
				fit_new = robot.get_fitness()	
				# BIG POINT - here we keep regardless if the change is better or not			

				if fit_new > best_fit:			# update learning curve
					best_fit = fit_new
					best_genome = robot.genome.copy()
				beam_fit[counter] = best_fit
				counter += 1
			# concat the populations and population fitnesses
			total_population = current_population + population
			total_population_fitness = current_population_fitness + population_fitness
			# sort the lists
			self.quick_sort(total_population_fitness , total_population , 0 , len(total_population) - 1)
			# keep the top half
			population = total_population[:self.population_size]
			population_fitness = total_population_fitness[:self.population_size]
		print(counter)
		np.savetxt("beam_genome.csv", best_genome, delimiter=",")
		np.savetxt("beam_learning.csv", beam_fit, delimiter=",")


class Simulator():
    # Place holder class: it returns a random fitness
    # TO DO - import the actual simulator class
    def calc_fitness(self, robot):
        fit = np.random.uniform(0 , 10000)
        robot.fitness = fit

if __name__ == '__main__':
    s = Simulator()

    alg = Beam_Search(100 , 10 , s)
    alg.run_algorithm()

