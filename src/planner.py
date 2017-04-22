import numpy as np

class Planner(object):
	
	def __init__(self, theta_s, theta_g, alpha):	
		self.s = theta_s
		self.g = theta_g
		self.alpha = alpha
		self.t_f = self.alpha*(linalg.norm(self.s-self.g)**2)

	def linear_path(self,t,curr_pos):
		"""
		Returns time-dependant configuratrion for straight line trajectory. 
		- Method: 	1st order time-parametrized function
		"""
		self.s = curr_pos
		self.t_f = self.alpha*(linalg.norm(self.s-self.g))

		theta = (self.g-self.s)*(1/self.t_f)*t + self.s

		print "alpha: " + str(self.alpha)
		print "total t: " + str(self.total_t)
		print "t_f: " + str(t_f)

		# if time after the final time, then just go to goal
		if t > self.t_f:
			theta = self.g

		return (self.t_f, theta)
