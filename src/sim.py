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
from utils import *

# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm by clicking on the plot near 
# the joint you want to affect. 
#--------------------------------------------------#

# Simulate basic straight-line trajectory following and
# torque computations
def basic_sim(xy_start, xy_goal, num_waypoints):
	# initialize default arm 
	arm2 = arm.TwoLinkArm()
	
	# set the starting configuration to start of trajectory 
	theta_start = arm2.inv_kin_ee(xy_start)
	arm2.theta = theta_start
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = get_line_traj(xy_start, xy_goal, num_waypoints)
	
	# convert trajectory from world to cspace with inverse kinematics
	theta_d = traj_to_cspace(xy_traj, arm2)
	dtheta_d = compute_dtheta(theta_d)
	ddtheta_d = compute_ddtheta(theta_d)
	
	# compute tau from the theta, dtheta, ddtheta
	tau_d = traj_tau_d(arm2, theta_d, dtheta_d, ddtheta_d, num_waypoints)
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
		arm_plot(plt, arm2, theta_d[i])
		# plot trajectory 
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')

		plt.pause(0.01)
	
	t = [x for x in range(num_waypoints)]
	
	# plot the tau's over time
	tau_plot(plt, tau_d, t)
	
	# plot velocities over time 
	vel_plot(plt, dtheta_d, t)
	
	plt.show()
	
def sim_human_correction(xy_start, xy_goal, num_waypoints, waypoint_H, tau_H):

	# initialize arm and trajectory given input
	# compute needed tau_d, theta_d, dtheta_d, ddtheta_d for trajectory
	# for t in each timestep,
	# 		# check if got to desired location - if not, then adjust tau
	# 		if current theta_a[t] != theta_d[t]:
	# 			compute dtheta_a[t], ddtheta)a[t], tau_a[t] 
	# 			M_a = compute_M(alpha, beta, delta, theta_a[t])
	# 			C_a = compute_C(alpha, beta, delta, theta_a[t], dtheta_a[t])
	# 			tau_H[t] = tau_d[t] - inv_dynamics(M_a, C_a, dtheta_a[t], ddtheta_a[t])
	#			
	#			# recompute tau_d[t+1] to get back to trajectory 
	#			tau_d[t] = inv_dynamics(M_a, C_a, dtheta_a[t], ddtheta_a[t])
	# 		apply tau_d[t] to get to new theta_d[t]

	# initialize default arm 
	arm2 = arm.TwoLinkArm()
	
	# set the starting configuration to start of trajectory 
	theta_start = arm2.inv_kin_ee(xy_start)
	arm2.theta = theta_start
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = get_line_traj(xy_start, xy_goal, num_waypoints)
	
	# convert trajectory from world to cspace with inverse kinematics
	theta_d = traj_to_cspace(xy_traj, arm2)
	dtheta_d = compute_dtheta(theta_d)
	ddtheta_d = compute_ddtheta(theta_d)
	
	# compute tau from the theta, dtheta, ddtheta
	tau_d = traj_tau_d(arm2, theta_d, dtheta_d, ddtheta_d, num_waypoints)
	# just sanity check the tau by computing ddtheta
	# ddtheta_d_verified = utils.traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints)

	# initialize graphing 
	fig = plt.figure(figsize=(10,10))
	for t in range(num_waypoints):
		plt.clf()
		# plot the end-effector position 
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
	
		# simulate applied human force applied at specific time
		if t == waypoint_H:
			
		
		ax.text(.05, .05, 'end-effector (x,y): ('+str(round(xy_traj[t][0],3))+','+str(round(xy_traj[t][1],3))+')', 
				horizontalalignment='left',
				verticalalignment='bottom',
				transform=ax.transAxes, 
				fontsize=15)
				
		#plot the arm 
		arm_plot(plt, arm2, theta_d[t])
		# plot trajectory 
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')

		plt.pause(0.01)
	
	
	