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

class UserClick():
	x = 0.0
	y = 0.0
	new_click = False
	def __init__(self, x,y):
		self.x = x
		self.y = y

	# get user input for updates to torques	
	def __call__(self, event):
		print "(user x,y): ", (event.xdata, event.ydata)
		print "(self x,y): ", (self.x, self.y)
		# Manipulate the data
		if event.xdata != self.x and event.ydata != self.y:
			self.new_click = True
			self.x = event.xdata
			self.y = event.ydata
		else:
			self.new_click = False

if __name__ == "__main__":
	# Constants
	l1 = 1 			# link 1 length 
	l2 = 1 			# link 2 length
	m1 = 1 		# link 1 mass 
	m2 = 0.5 		# link 2 mass
	dt = 0.01 		# sim time step

	# Init Conditions
	theta1 = pi/8 	# shoulder theta
	theta2 = pi/8	# joint theta
	dtheta1 = 0		# shoulder angular velocity 
	dtheta2 = 0		# joint angular velocity 

	fig = plt.figure(figsize=(10,10))
	plt.clf()
	ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
	ax.grid()
	plt.hold(True)

	# initialize UserClick
	user_click = UserClick(0, 0)

	t = 0
	tau1 = 0 # torque on link1 (shoulder)
	tau2 = 0 # torque on line2 (elbow)

	r1 = 0.5 # distance from first joint to center of mass for link 1
	r2 = 0.5 # distance from second joint to center of mass for link 2

	# compute constants 
	alpha = m1*r1**2 + m2*(l1**2 + r2**2)
	beta = m2*l1*r2
	delta = m2*r2**2

	i = 0
	while(1 is 1):
		print "\n(tau1, tau2): ", (tau1, tau2)
		
		# looping until user quits 
		t += dt
		
		c2 = cos(theta2)
		s2 = sin(theta2)
		
		# compute inertial forces due to acceleration of joints
		M00 = alpha + 2*beta*c2
		M01 = delta + beta*c2
		M10 = delta + beta*c2
		M11 = delta
		
		# compute coriolis and centrifugal forces
		C00 = -beta*s2*dtheta1 
		C01 = -beta*s2*(dtheta1 + dtheta2)
		C10 = beta*s2*dtheta1 
		C11 = 0
		
		H1 = -beta*s2*dtheta1*dtheta2 - (1/2)*beta*s2*(dtheta2**2)
		H2 = (1/2)*beta*s2*(dtheta1**2)
		
		# compute ddtheta's from tau1,tau2, and the inertial/corioilis forces
		ddtheta2 = (H2*M00 - H1*M10 -M00*tau2 + M10*tau1) / (M01**2 - M00*M11)
		ddtheta1 = (-H2 + tau2 - M11*ddtheta2) / M10

		print "ddtheta1: ", ddtheta1
		print "ddtheta2: ", ddtheta2
		
		# update dtheta and theta
		dtheta1_new = dtheta1 + ddtheta1*dt
		dtheta2_new = dtheta2 + ddtheta2*dt
		theta1_new = theta1 + dtheta1_new*dt
		theta2_new = theta2 + dtheta2_new*dt
		
		# move to new timestep
		dtheta1 = dtheta1_new
		dtheta2 = dtheta2_new
		
		print "dtheta1: ", dtheta1
		print "dtheta2: ", dtheta2
		
		theta1 = theta1_new
		theta2 = theta2_new
		
		print "theta1: ", theta1
		print "theta2: ", theta2
		
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
		x1 = r1*cos(theta1)
		y1 = r1*sin(theta1)

		x2 = x1 + r2*cos(theta1 + theta2) 
		y2 = y1 + r2*sin(theta1 + theta2)
		
		print "(x1,y1): ", (x1, y1)
		print "(x2,y2): ", (x2, y2)
			
		cid = fig.canvas.mpl_connect('button_press_event', user_click)
		print "new_click: ", user_click.new_click

		if(user_click.new_click):
			user_x = user_click.x
			user_y = user_click.y
			
			# distance to midpoint of l1 and l2
			dist_to_l1 = math.hypot(user_x-(x1/2), user_y-(y1/2))
			dist_to_l2 = math.hypot(user_x-((x2+x1)/2), user_y-((y2+y1)/2))
			
			if dist_to_l1 < dist_to_l2: # user interacting with shoulder
				value_l1 = (x1-0)*(user_y-0) - (user_x - 0)*(y1 - 0)
				if value_l1 > 0:
					print "SHOULDER torque on LEFT"
					# torque applied from left 
					tau1 -= 0.1
				else:
					print "SHOULDER torque on RIGHT"
					# torque applied from right
					tau1 += 0.1
			else: # user interacting with elbow
				value_l2 = (x2-x1)*(user_y-x1) - (user_x - x1)*(y2 - y1)
				if value_l2 > 0:
					print "ELBOW torque on LEFT"
					# torque applied from left 
					tau2 -= 0.1
				else:
					print "ELBOW torque on RIGHT"
					# torque applied from right
					tau2 += 0.1
			user_click.new_click = False
		
		plt.clf()
		plt.hold(True)
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
		ax.grid()
		
		plt.plot([0, x1], [0, y1], 'b', linewidth=4)
		plt.plot([x1, x2], [y1, y2], 'b', linewidth=4)
		
		# plot joints
		plt.plot(0,0,'ko')
		plt.plot(x1,y1,'ko'); 
		plt.plot(x2,y2,'ko') 
		
		#plt.show(block=False)
		plt.pause(0.001)
		