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

import time
import controller
import planner
from planner import *
from controller import *

# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm by clicking on the plot near 
# the joint you want to affect. 
#--------------------------------------------------#

def PID_control(control, target_pos, pos):
	"""
	Return a control torque based on PID control
	"""
	error = (target_pos - pos)

	return -control.update_PID(error)

def update_target_pos(curr_pos, plan):
	"""
	Takes the current position of the robot. Determines what the next
	target position to move to should be depending on:
	- if robot is moving to start of desired trajectory or 
	- if robot is moving along the desired trajectory 
	"""
	delta_t = 0.01;
	(T, target_pos) = plan.linear_path(delta_t, curr_pos)
	return target_pos

def pid_sim(xy_start, xy_goal):
	"""
	Simulates a PID controller operating on 2-link arm and a straight
	line trajectory in C-Space. Human can press left and right arrows keys
	to simulate force applied to robot's end-effector in the direction of the 
	trajectory or against the trajectory direction.
	"""
	num_waypoints = 20

	# initialize default arm 
	arm2 = arm.TwoLinkArm()

	# set the starting configuration to start of trajectory 
	theta_start = arm2.inv_kin_ee(xy_start)
	theta_goal = arm2.inv_kin_ee(xy_goal)
	arm2.theta = theta_start
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = get_line_traj(xy_start, xy_goal, num_waypoints)

	dim = 2 # 2-dimensional system for 2 link arm
	alpha = 1.0

	dt = 0.001

	# set up the control input command
	torque = np.eye(dim) 

	# set up the path planner
	plan = planner.PathPlanner(theta_start, theta_goal, alpha)

	# set up the controller
	p_gain = 1.0;
	i_gain = 1.0;
	d_gain = 1.0;
	P = p_gain*np.eye(dim)
	I = i_gain*np.eye(dim)
	D = d_gain*np.eye(dim)
	control = controller.Controller(P,I,D,0,0,dim)

	# keep track of starting time of trajectory execution
	path_start_T = time.time() 

	# store this for computing velocity
	prev_theta = arm2.theta

	# store this for computing deltaT timestep
	last_time = time.time()

	fig = plt.figure(figsize=(10,10))

	while(True):	

		curr_pos = arm2.theta
		print "goal_pos: (" + str(theta_goal[0][0]) + ", " +  str(theta_goal[1][0]) + ")"
		print "curr_pos: (" + str(curr_pos[0][0]) + ", " +  str(curr_pos[1][0]) + ")"

		# update target position along trajectory
		target_pos = theta_goal #update_target_pos(curr_pos, plan)
		print "target_pos: (" + str(target_pos[0][0]) + ", " +  str(target_pos[1][0]) + ")"

		# compute torque needed to move towards target pos
		torque = PID_control(control, target_pos, curr_pos)
		print "torque: (" + str(torque[0][0]) + ", " +  str(torque[1][0]) + ")"

		# compute timestep
		#curr_time = time.time()
		#dt = curr_time - last_time
		#last_time = curr_time

		# apply torque to arm and update the arm's position and velocity
		arm2.apply_torque(torque, dt)

		# update velocity and position 
		#arm2.dtheta[0] = (arm2.theta[0][0] - prev_theta[0][0])/dt #TODO THIS IS WRONG!
		#arm2.dtheta[1] = (arm2.theta[1][0] - prev_theta[1][0])/dt

		prev_theta = arm2.theta		

		ee_pos = arm2.fwd_kin_ee()
		
		plt.clf()
		# plot the end-effector position 
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
		#ax.text(.05, .05, 'TRAJ ee(x,y): ('+str(round(xy_traj[t][0],3))+','+str(round(xy_traj[t][1],3))+')', 
		#		horizontalalignment='left',
		#		verticalalignment='bottom',
		#		transform=ax.transAxes, 
		#		fontsize=15)
		ax.text(.05, .0, 'ACTUAL ee(x,y): ('+str(round(ee_pos[0],3))+','+str(round(ee_pos[0],3))+')', 
				horizontalalignment='left',
				verticalalignment='bottom',
				transform=ax.transAxes, 
				fontsize=15)
				
		#plot the arm 
		arm2.plot(plt)
		# plot trajectory 
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')
		
		plt.pause(0.01)

