import numpy as np 


class hill_climber():

	def __init__(self, num_iterations , robot):
		self.num_iterations = num_iterations
		self.robot = robot


	def mutate(self , genome):
		"""
		This makes a mutation (very small random change) The a single robot's genome
		Arg:
			genome: a nxm numpy array
		"""


		num_rows = genome.shape[0]
		num_cols = genome.shape[1]

		row = np.random.randint(num_rows)			# get the indices for rows , columns
		col = np.random.randint(num_cols)			#              "  "
		adjustment = np.random.uniform(0.9 , 1.1)		# select random value to multiply element with
		old_val = genome[row , col]						
		genome[row , col] = genome[row , col] * adjustment			# perform multiplication

		return [row , col] , old_val 					# return location of mutation and old value
														#  (In case we need to change back)


	def run_algorithm():

		"""
		* this function performs a parametric optimization hill climber algorithm on a single robot.
		* it will save the top performing robot, as well as learning curve 

		"""
		robot = self.robot 
		num_climb = self.num_iterations

		hc_fit = np.zeros(num_climb+1)								# create an array to store the learning curve data


		robot.build_genome()										# build the genome

		best_fit = calc_fitness()					# get the fitness of the first robot
		best_array = robot.genome 					# initialize the best array to keep track
		hc_fit[0] = best_fit						# place it as the first value in the learning curve array

		for i in range(1 ,num_climb + 1):			# perform mutations equal to num_Climb
			print('mut' , i)						
			mut_loc , old_val = self.mutate(robot.genome)		# Mutation:  Keep track of mut location and previous vals

			fit_new = calc_fitness( robot , sim)	# calculate new fitness
			
			if fit_new > best_fit:					# if better update the top fitness, and top performing genome
				best_fit = fit_new					
				best_array = np.copy(robot.genome)

			else:									# if worse, revert the genome back to its previous state. 
				robot.genome[mut_loc[0] , mut_loc[1]] = old_val				# only need to adjust genome
			hc_fit[i] = best_fit											# robot is rebuilt prior to fitness calculation each time
			
		np.savetxt("hc2_robot.csv", best_array, delimiter=",")
		np.savetxt("hc2_learning.csv", hc_fit, delimiter=",")

