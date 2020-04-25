import numpy as np 


class evolutionary_algorithm():

	def __init__(self , num_rows , num_cols , population_size  , robot):
		self.num_rows = num_rows
		self.num_cols = num_cols
		self.population_size = population_size
		self.robot = robot


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
	        

	def make_population(self , num):
		"""
		This method builds a population of robot genomes
		Args:
			num: The number of robot genomes in your desired population
				+ Keeping this as an arg so we can build variable sized populations
		"""


		pop = []		 	# initialize an empty list
		for i in range(num):
			self.robot.make_genome()
			pop += [robot.genome]
		

	def mutate(self , genome):
		"""
		This makes a mutation (very small random change) The a single robot's genome
		Arg:
			genome: a nxm numpy array
		"""

		row = np.random.randint(self.num_rows)			# get the indices for rows , columns
		col = np.random.randint(self.num_cols)			#              "  "
		adjustment = np.random.uniform(0.9 , 1.1)		# select random value to multiply element with
		old_val = genome[row , col]						
		genome[row , col] = genome[row , col] * adjustment			# perform multiplication

		return [row , col] , old_val 					# return location of mutation and old value
														#  (In case we need to change back)

	def do_crossover(self , genome1 , genome2):
		"""
		This function performs a crossover between two robot genomes It will do so in a random direction
		and make two random cuts.
		Args:
			genome1, genome2: 2 numpy arrays of identical shape. These are the specific robot genomes
		Output:
			a new genome, which will be used for the child robot
		"""	

		dimension = np.random.randint(2)    # do we cut over the rows or cols

		if dimension == 0:										# if cutting on rows
			cut1 = np.random.randint(self.num_rows)				# get both cuts, make sure they are different
			cut2 = np.random.randint(self.num_rows)				#           "  "
			while cut1 == cut2:	
				cut1 = np.random.randint(self.num_rows)
			first = min(cut1 , cut2)							# get the smaller and larget cuts
			second = max(cut1 , cut2)
			new_genome = np.vstack((genome1[ :first , :] , genome2[first : second , :] , genome1[ second: , :]))	# make new genome
			
		else:													# if cutting on columns
			cut1 = np.random.randint(self.num_cols)				# get both cuts, make sure they are different
			cut2 = np.random.randint(self.num_cols)
			while cut1 == cut2:	
				cut1 = np.random.randint(self.num_cols)
			
			first = min(cut1 , cut2)							# get the smaller and larget cuts
			second = max(cut1 , cut2)
			new_genome = np.hstack((genome1[:, :first] , genome2[:, first : second] , genome1[: , second :]))	# make new genome
			

		return new_genome					# return the child genome




	def run_algorithm(self , robot ):
		"""
		This method will run the evolutionary algorithm itself
		Args:
			robot: This is the robot object to be used in the simulation - could be taken in as an arg for the class instead
		Output:
			This outputs two csv files. 
			1. "ea_robot_genome.csv" represents the best performing genome
			2. "ea_learning.csv" is data for the algorithm's learning curve

		"""
		robot = self.robot

		# ______  Step1: Population generation _________
		parent_size = self.population_size / 2
		parents = self.make_population(parent_size)		# make the population of parents - These are genomes

		overall_fitness = np.zeros(evals + 1)		# learning curve array
		ticker = 0									# keeps track of which robot eval we are on
		population_fitness = [0] * population_size 		# list used to keep track of the fitness of each robot in the population
		
		# ______  Step2: Crossover _________
		population = parents.copy() 							# make a shallow copy of the parents array
		for k in range(parent_size):							# select 2 random parents
			p1 = np.random.randint(population_size/2)			#		....
			p2 = np.random.randint(population_size/2)			#		....
											
			while p2 == p1:										# make sure they are not the same robot					
				p2 = np.random.randint(population_size/2)		# note: if pop size = 2; stuck here forever
			
			pr1 = parents[p1]									# perform a 2 cut crossover
			pr2 = parents[p2]									# 		....
			c = crossover(pr1 , pr2)							# 		....
			population = population + [c]						# add the child robot genomes to the population
		
		for z in range(len(population_fitness)):				# update the population fitness array 
			if population_fitness[z] ==0:						# only do this if there are zeros 
				genome = population[z]														# select the robot
				
				robot.genome = genome 							# Set the robot's attribute to be the current genome
				population_fitness[z] = robot.calc_fitness(genome , sim)    # THIS METHOD MAY NEED TO BE CHANGED - this should run the simulation

		for m in range(num_generations):			# perform the EA
			print('generation: ' , m)
			# ______  Step3: Mutation _________

			
			for j in range(num_mutations):
				print('mut' , j)
				for p in range(population_size):					# mutate each robot in the population
					genome = population[p]							#			....

					if ticker == 0:									# if this is the first evaluation, add fitness as first element to learning curve array
						overall_fitness[ticker] = population_fitness[p]			#			....
						ticker += 1												#Keep track of evaluation number
					
					mut_loc , old_val = self.mutate(genome)		# Mutation:  Keep track of mut location and previous vals
					
					robot.genome = genome 							# Set the robot's attribute to be the current genome
					fit_new = robot.calc_fitness(genome , sim)      # THIS METHOD MAY NEED TO BE CHANGED - this should run the simulation
						
					if fit_new > population_fitness[p]:				# if higher fitness then this robot's previous best : update
						population_fitness[p]= fit_new				# 			....
						
					else:
						genome[mut_loc[0] , mut_loc[1]] = old_val			# if lower, revert the genome to it's previous state
					
					if overall_fitness[ticker - 1] < population_fitness[p]:		# if the best overall robot thus far
						overall_fitness[ticker] = population_fitness[p]			# add this as next value for the learning curve
						best_array = np.copy(genome)						# update the best robot's genome
					else:														# if lower, push the previous best score forward in learning curve
						overall_fitness[ticker] = overall_fitness[ticker - 1]
					ticker +=1													# end of mutation, add one to robot evaluation ticker

	# __________ Truncation Selection ____________
			self.quick_sort( population_fitness , population )			# sort the population according to fitness rank ( sorts both population and population_fintess in tandem)

			parents = population[: parent_size] 				# perform truncation selection (choose top half)
			second_half = [0] * parent_size						# 	array of zeros
			population_fitness = population_fitness[: parent_size] + second_half		# 	 keep first half (second half are zeros)

	# __________ Complete algorithm ______________

		a = population[0]					# extract the best performing robot
		np.savetxt("ea_robot_genome.csv", a, delimiter=",")
		np.savetxt("ea_learning.csv", overall_fitness, delimiter=",")


if __name__ == '__main__':
	EA = evolutionary_algorithm(num_rows = 2 , num_cols = 2 , population_size = 100)
	EA.run_algorithm()