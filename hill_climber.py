import numpy as np 


class hill_climber():

	def __init__(self, num_iterations , robot):
		self.num_iterations = num_iterations
		self.robot = robot


	def mutate():
		pass



	def run_algorithm():

		"""
		* this function performs a parametric optimization hill climber algorithm on a single robot.
		* it will save the top performing robot, as well as learning curve 

		"""
		robot = self.robot 
		num_climb = self.num_iterations

		hc_fit = np.zeros(num_climb+1)								# create an array to store the learning curve data
		# keep track of progress
		best_fit = 0												# init the best fit as zero
		best_array = np.empty(8)									# init a random array as the best robot
		# make a random robot	

		robot.build_genome()										# build the genome

		best_fit = calc_fitness()					# get the fitness of the first robot
		hc_fit[0] = best_fit						# place it as the first value in the learning curve array

		for i in range(1 ,num_climb + 1):			# perform mutations equal to num_Climb
			print('mut' , i)						
			mut_loc , old_val = robot.mutate_genome()		# Mutation:  Keep track of mut location and previous vals
													# rebuild the robot with the new (mutated) genome
			robot.remove_old()						# remove old springs and masses
			robot.set_tetr( np.array((0 , 0 , 0)) , np.array((0 , 0 , 0)))		# set the robot to pos(0 , 0 , 0)
			robot.build_robot()						# builds the robot
													
			mass_list = robot.mass_list				# center the robot
			com = robot.get_com(mass_list)			# 		....
			robot.center_object(com)				# 		....

			fit_new = calc_fitness( robot , sim)	# calculate new fitness
			if fit_new > best_fit:					# if better update the top fitness, and top performing genome
				best_fit = fit_new					
				best_array = np.copy(robot.genome)

			else:									# if worse, revert the genome back to its previous state. 
				robot.genome[mut_loc[0] , mut_loc[1]] = old_val				# only need to adjust genome
			hc_fit[i] = best_fit											# robot is rebuilt prior to fitness calculation each time
			
		np.savetxt("hc2_robot.csv", best_array, delimiter=",")
		np.savetxt("hc2_learning.csv", hc_fit, delimiter=",")

