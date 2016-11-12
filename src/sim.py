import numpy as np
import matplotlib.pyplot as plt
import sys

import pylab
import matplotlib.image as mpimg
import matplotlib.animation as animation

import numpy.linalg
from numpy.linalg import *

import math
from math import  *

import arm 
import utils

# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm by clicking on the plot near 
# the joint you want to affect. 
#--------------------------------------------------#

if __name__ == "__main__":

	# initialize default arm 
	arm = arm.Arm()

	M = utils.compute_M(arm.alpha, arm.beta, arm.delta, arm.theta)
	print "M: ", M 
	C = utils.compute_C(arm.alpha, arm.beta, arm.delta, arm.theta, arm.dtheta)
	print "C: ", C

	arm.tau = np.array([[-1.6],[1.8]])
	ddtheta = utils.fwd_dynamics(M, C, arm.tau, arm.dtheta)
	print "ddtheta: ", ddtheta
	tau_verified = utils.inv_dynamics(M, C, arm.dtheta, ddtheta)
	print "tau_verified: ", tau_verified

	fig = plt.figure(figsize=(10,10))
	plt.clf()
	ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
	ax.grid()
	plt.hold(True)
	
	# start/end for trajectory
	arm_pos = arm.fwd_kin_ee()
	SX = arm_pos[0]; SY = arm_pos[1]
	GX = 0.0; GY = 1.5
	num_waypoints = 5
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = utils.get_line_traj((SX,SY), (GX,GY), num_waypoints)
	print "xy_traj: ", xy_traj 

	# convert trajectory from world to cspace with inverse kinematics
	theta_d = utils.traj_to_cspace(xy_traj, arm)
	print "configuration traj: ", theta_d

	dtheta_d = utils.compute_dtheta(theta_d)
	ddtheta_d = utils.compute_ddtheta(theta_d)

	print "dtheta_d: ", dtheta_d
	print "ddtheta_d: ", ddtheta_d
	
	tau_d = utils.traj_tau_d(arm, theta_d, dtheta_d, ddtheta_d, num_waypoints)

	ddtheta_d_verified = utils.traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints)

	# get final trajectory
	theta_traj = np.zeros((num_waypoints, 2))

	for i in range(num_waypoints):
		plt.clf()
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
	
		x1 = arm.l1*cos(theta_d[i][0])
		y1 = arm.l1*sin(theta_d[i][0])

		x2 = x1 + arm.l2*cos(theta_d[i][0] + theta_d[i][1])
		y2 = y1 + arm.l2*sin(theta_d[i][0] + theta_d[i][1])
	
		plt.plot([0, x1], [0, y1], 'b', linewidth=5)
		plt.plot([x1, x2], [y1, y2], 'b', linewidth=5)
	
		# plot joints
		plt.plot(0,0,'ko')
		plt.plot(x1, y1, 'ko') 
		plt.plot(x2, y2, 'ko')
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')

		plt.pause(0.01)
	
