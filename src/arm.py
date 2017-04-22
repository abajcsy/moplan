import numpy as np
import matplotlib.pyplot as plt
import sys

import numpy.linalg
from numpy.linalg import *

import math
from math import  *

import utils

class TwoLinkArm():
	# Constants
	l1 = 1.0 			# link 1 length 
	l2 = 1.0 			# link 2 length
	m1 = 1.0 			# link 1 mass 
	m2 = 1.0 			# link 2 mass

	#r1 = 1.0 		# distance from first joint to center of mass for link 1
	#r2 = 1.0 	 	# distance from second joint to center of mass for link 2

	# Init Conditions
	theta = np.zeros((2,1)) 		# [shoulder theta, joint theta]
	dtheta = np.zeros((2,1))		# [shoulder angular vel, joint angular vel] 
	
	tau = np.zeros((2,1))			# [torque on link1 (shoulder), torque on link2 (elbow)]
	
	dt = 0.01 	# sim time step

	# constants in computation of inertia and coriolis forces matrices  
	alpha = 0
	beta = 0
	delta = 0
	
	# compute x,y position of arm's end-effector and elbow
	ee = np.zeros((2,1)) 
	elbow = np.zeros((2,1)) 

	# inertial matrix
	M = np.zeros((2,2))
	# coriolis forces matrix
	C = np.zeros((2,2))

	def __init__(self):
		# set up links and masses
		self.l1 = 1.0
		self.l2 = 1.0
		self.m1 = 0.5
		self.m2 = 0.5

		# set starting configuration
		self.theta[0] = 0.0
		self.theta[1] = 0.0
		# set dtheta as default 0 to begin with
		self.dtheta[0] = 0.0
		self.dtheta[1] = 0.0

		# set (x,y) ee and elbow position 
		self.ee[0] = self.l1*cos(self.theta[0])
		self.ee[1] = self.l1*sin(self.theta[0])
		self.elbow[0] = self.ee[0] + self.l2*cos(self.theta[0] + self.theta[1])
		self.elbow[1] = self.ee[1] + self.l2*sin(self.theta[0] + self.theta[1])

		# compute constants 
		self.alpha = self.m1*self.l1**2 + self.m2*(self.l1**2 + self.l2**2)
		self.beta = self.m2*self.l1*self.l2
		self.delta = self.m2*self.l2**2

		# update M and C matrices given theta, dtheta information
		self.M = utils.compute_M(self.alpha, self.beta, self.delta, self.theta)
		self.C = utils.compute_C(self.alpha, self.beta, self.delta, self.theta, self.dtheta)

	# get forward kinematics of end-effector (end of link2)
	def fwd_kin_ee(self):
		x2 = self.l1*cos(self.theta[0]) + self.l2*cos(self.theta[0] + self.theta[1])
		y2 = self.l1*sin(self.theta[0]) + self.l2*sin(self.theta[0] + self.theta[1])
		return np.array([[x2],[y2]])

	# get forward kinematics of elbow elbow (end of link 1)
	def fwd_kin_elbow(self):
		x1 = self.l1*cos(self.theta[0])
		y1 = self.l1*sin(self.theta[0])
		return np.array([[x1], [y1]])

	# get inverse kinematics of end-effector to compute angles
	# needed in order to get to ee to (x,y) pos
	def inv_kin_ee(self, pos):
		x = pos[0]
		y = pos[1]

		c2 = (x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2)
	
		try: 
			s2 = sqrt(1 - c2**2)
		except:
			print "\n--------------------------------"
			print "WARNING: TRAJECTORY POINT ", (x,y) 
			print "OUTSIDE OF ROBOT'S WORKSPACE"
			print "--------------------------------\n"
			return None

		t2 = atan2(s2, c2)
		k1 = self.l1 + self.l2*c2
		k2 = self.l2*s2
		t1 = atan2(y,x) - atan2(k2, k1)
		
		config = np.zeros((2,1))
		config[0] = t1
		config[1] = t2

		return config
		
	def plot(self, plt):
		ee = self.fwd_kin_ee()
		elbow = self.fwd_kin_elbow()
		
		plt.plot([0, elbow[0]], [0, elbow[1]], 'g', linewidth=5)
		plt.plot([elbow[0], ee[0]], [elbow[1], ee[1]], 'b', linewidth=5)

		# plot joints
		plt.plot(0,0,'ko')
		plt.plot(elbow[0], elbow[1], 'ko') 
		plt.plot(ee[0], ee[1], 'ko')
		
