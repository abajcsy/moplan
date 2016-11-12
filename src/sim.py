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
	
	# start/end for trajectory
	xy_start = [1.5, -1.0]
	xy_goal = [-1.0, 1.5]
	num_waypoints = 20
	
	# set the starting configuration to start of trajectory 
	theta_start = arm.inv_kin_ee(xy_start)
	arm.theta = theta_start
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = utils.get_line_traj(xy_start, xy_goal, num_waypoints)
	
	# convert trajectory from world to cspace with inverse kinematics
	theta_d = utils.traj_to_cspace(xy_traj, arm)
	dtheta_d = utils.compute_dtheta(theta_d)
	ddtheta_d = utils.compute_ddtheta(theta_d)
	
	# compute tau from the theta, dtheta, ddtheta
	tau_d = utils.traj_tau_d(arm, theta_d, dtheta_d, ddtheta_d, num_waypoints)
	# just sanity check the tau by computing ddtheta
	# ddtheta_d_verified = utils.traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints)

	# initialize graphing 
	fig = plt.figure(figsize=(10,10))
	for i in range(num_waypoints):
		plt.clf()
		
		# plot the end-effector position 
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
		ax.text(.05, .05, 'end-effector (x,y): ('+str(round(xy_traj[i][0],3))+','+str(round(xy_traj[i][1],3))+')', 
				horizontalalignment='left',
				verticalalignment='bottom',
				transform=ax.transAxes, 
				fontsize=15)
				
		#plot the arm 
		utils.arm_plot(plt, arm, theta_d[i])
		# plot trajectory 
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')

		plt.pause(0.01)
	
	t = [x for x in range(num_waypoints)]
	
	# plot the tau's over time
	tau_plot(plt, tau, t)
	
	# plot velocities over time 
	vel_plot(plt, dtheta_d, t)
	
	plt.show()
	