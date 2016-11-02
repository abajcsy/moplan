import numpy as np
import matplotlib.pyplot as plt

import pylab
import matplotlib.image as mpimg
import matplotlib.animation as animation

import math
from math import  *
# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm using keys:
# 		q/w - shoulder
#		o/p - elbow 
#--------------------------------------------------#

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

# figure(1); 
# clf; axis([-2.5 2.5 -.2 2.5]); axis image; hold on; 
# set(gcf,'doublebuffer','on'); set(gcf,'KeyPressFcn','keydown=1;'); 

fig = plt.figure(figsize=(10,10))
plt.clf()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
ax.grid()
plt.hold(True)

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
	
	# get user input for updates to torques
	
	
	
	
	
	tau1 += 0.1
	tau2 += 0.1
	
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
	