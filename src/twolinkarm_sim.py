import numpy as np
import matplotlib.pyplot as plt
import sys

import pylab
import matplotlib.image as mpimg
import matplotlib.animation as animation

import math
from math import  *

# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm by clicking on the plot near 
# the joint you want to affect. 
#--------------------------------------------------#

class Arm():
	# Constants
	l1 = 1 			# link 1 length 
	l2 = 1 			# link 2 length
	m1 = 1 		# link 1 mass 
	m2 = 0.5 		# link 2 mass

	# Init Conditions
	theta1 = pi/8 	# shoulder theta
	theta2 = pi/8	# joint theta
	dtheta1 = 0		# shoulder angular velocity 
	dtheta2 = 0		# joint angular velocity 
	
	tau1 = 0 # torque on link1 (shoulder)
	tau2 = 0 # torque on line2 (elbow)

	r1 = 0.5 # distance from first joint to center of mass for link 1
	r2 = 0.5 # distance from second joint to center of mass for link 2
	
	dt = 0.01 		# sim time step

	# compute constants 
	alpha = m1*r1**2 + m2*(l1**2 + r2**2)
	beta = m2*l1*r2
	delta = m2*r2**2
	
	# compute x,y position of arm
	x1 = r1*cos(theta1)
	y1 = r1*sin(theta1)

	x2 = x1 + r2*cos(theta1 + theta2) 
	y2 = y1 + r2*sin(theta1 + theta2)

	#def __init__(self):
	
	def update(self):
		c2 = cos(self.theta2)
		s2 = sin(self.theta2)
		
		# compute inertial forces due to acceleration of joints
		M00 = self.alpha + 2*self.beta*c2
		M01 = self.delta + self.beta*c2
		M10 = self.delta + self.beta*c2
		M11 = self.delta
		
		# compute coriolis and centrifugal forces
		C00 = -self.beta*s2*self.dtheta1 
		C01 = -self.beta*s2*(self.dtheta1 + self.dtheta2)
		C10 = self.beta*s2*self.dtheta1 
		C11 = 0
		
		H1 = -self.beta*s2*self.dtheta1*self.dtheta2 - (1/2)*self.beta*s2*(self.dtheta2**2)
		H2 = (1/2)*self.beta*s2*(self.dtheta1**2)
		
		# compute ddtheta's from tau1,tau2, and the inertial/corioilis forces
		ddtheta2 = (H2*M00 - H1*M10 -M00*self.tau2 + M10*self.tau1) / (M01**2 - M00*M11)
		ddtheta1 = (-H2 + self.tau2 - M11*ddtheta2) / M10

		print "ddtheta1: ", ddtheta1
		print "ddtheta2: ", ddtheta2
		
		# update dtheta and theta
		dtheta1_new = self.dtheta1 + ddtheta1*self.dt
		dtheta2_new = self.dtheta2 + ddtheta2*self.dt
		theta1_new = self.theta1 + dtheta1_new*self.dt
		theta2_new = self.theta2 + dtheta2_new*self.dt
		
		# move to new timestep
		self.dtheta1 = dtheta1_new
		self.dtheta2 = dtheta2_new
		
		print "dtheta1: ", self.dtheta1
		print "dtheta2: ", self.dtheta2
		
		self.theta1 = theta1_new
		self.theta2 = theta2_new
		
		print "theta1: ", self.theta1
		print "theta2: ", self.theta2
		
		# enforce joint constraints 
		if (1 == 2): #just for testing now
			if theta1 <= -0.1: # shoulder too far right 
				theta1 = -0.1 
				dtheta1 = 0 
				ddtheta1 = 0 
			elif theta1 >= pi-0.2: #shoulder too far left 
				theta1 = pi-0.2 
				dtheta1 = 0
				ddtheta1 = 0 

			if theta2 <= 0: # elbow locks extended 
				theta2 = 0
				dtheta2 = 0
				ddtheta2 = 0 
			elif theta2 >= pi-0.1: # forearm hits upper arm 
				theta2 = pi-0.1
				dtheta2 = 0
				ddtheta2 = 0 
				
		# compute new x,y position of arm
		self.x1 = self.r1*cos(self.theta1)
		self.y1 = self.r1*sin(self.theta1)

		self.x2 = self.x1 + self.r2*cos(self.theta1 + self.theta2) 
		self.y2 = self.y1 + self.r2*sin(self.theta1 + self.theta2)
		
		print "(x1,y1): ", (self.x1, self.y1)
		print "(x2,y2): ", (self.x2, self.y2)


def get_line_traj((SX,SY), (GX,GY), num_waypoints):
	traj = np.zeros((2,num_waypoints))
	traj[0][0] = SX
	traj[1][0] = SY
	dist_x = GX-SX
	dist_y = GY-SY

	for i in range(1,num_waypoints):
		traj[0][i] = traj[0][i-1] + dist_x/(num_waypoints-1)
		traj[1][i] = traj[1][i-1] + dist_y/(num_waypoints-1)
		
	return traj


if __name__ == "__main__":

	arm = Arm()
	
	fig = plt.figure(figsize=(10,10))
	plt.clf()
	ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
	ax.grid()
	plt.hold(True)

	# generate trajectory
	SX = 0.9
	SY = 0.5
	GX = -0.9
	GY = 0.5
	num_waypoints = 10
	traj = get_line_traj((SX,SY), (GX,GY), num_waypoints)
	plt.plot(traj[0],traj[1],'ro')
	
	t = 0
	i = 0
	while(1 is 1):
		print "\n(tau1, tau2): ", (arm.tau1, arm.tau2)
		
		# looping until user quits 
		t += arm.dt
		
		# update arm position given changes in it's state
		arm.update()
		
		# simulate applied torque 
		arm.tau1 += 0.1
			
		plt.clf()
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
		ax.grid()
		
		plt.plot([0, arm.x1], [0, arm.y1], 'b', linewidth=5)
		plt.plot([arm.x1, arm.x2], [arm.y1, arm.y2], 'b', linewidth=5)
		
		# plot joints
		plt.plot(0,0,'ko')
		plt.plot(arm.x1, arm.y1, 'ko') 
		plt.plot(arm.x2, arm.y2, 'ko')
		
		# plot trajectory waypoints
		plt.plot(traj[0],traj[1],'ro')
		
		#plt.show(block=False)
		plt.pause(0.001)
		
