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
	arm_pos = arm.fwd_kin_ee()
	SX = arm_pos[0]; SY = arm_pos[1]
	GX = 0.0; GY = 1.5
	num_waypoints = 20
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = utils.get_line_traj((SX,SY), (GX,GY), num_waypoints)

	# convert trajectory from world to cspace with inverse kinematics
	theta_d = utils.traj_to_cspace(xy_traj, arm)
	dtheta_d = utils.compute_dtheta(theta_d)
	ddtheta_d = utils.compute_ddtheta(theta_d)
	
	# compute tau from the theta, dtheta, ddtheta
	tau_d = utils.traj_tau_d(arm, theta_d, dtheta_d, ddtheta_d, num_waypoints)
	# just sanity check the tau by computing ddtheta
	ddtheta_d_verified = utils.traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints)

	# initialize graphing 
	fig = plt.figure(figsize=(10,10))
	for i in range(num_waypoints):
		left = .05
		bottom = .05
		plt.clf()
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
		ax.text(left, bottom, 'end-effector (x,y): ('+str(round(xy_traj[i][0],3))+','+str(round(xy_traj[i][1],3))+')', 
			horizontalalignment='left',
			verticalalignment='bottom',
			transform=ax.transAxes, 
			fontsize=15)
	
		x1 = arm.l1*cos(theta_d[i][0])
		y1 = arm.l1*sin(theta_d[i][0])

		x2 = x1 + arm.l2*cos(theta_d[i][0] + theta_d[i][1])
		y2 = y1 + arm.l2*sin(theta_d[i][0] + theta_d[i][1])
	
		plt.plot([0, x1], [0, y1], 'g', linewidth=5)
		plt.plot([x1, x2], [y1, y2], 'b', linewidth=5)
	
		# plot joints
		plt.plot(0,0,'ko')
		plt.plot(x1, y1, 'ko') 
		plt.plot(x2, y2, 'ko')
		plt.plot(xy_traj[:,0],xy_traj[:,1],'ro')

		plt.pause(0.01)
	
	# plot the tau's over time
	t = [x for x in range(num_waypoints)]
	fig2 = plt.figure(figsize=(15, 8))
	ax2 = fig2.add_subplot(111)
	ax2.grid()
	ax2.plot(t, tau_d[:,0], 'g', linewidth=2, label='Tau1')
	ax2.plot(t, tau_d[:,1],'b', linewidth=2, label='Tau2')
	
	# Now add the legend with some customizations.
	legend = ax2.legend(loc='upper right', shadow=True)
	ax2.set_title('Torque Applied at Waypoints')
	ax2.set_xlabel('Waypoint')
	ax2.set_ylabel('Torque')

	
	# plot velocities over time 
	fig3 = plt.figure(figsize=(15, 8))
	ax3 = fig3.add_subplot(111)
	ax3.grid()
	ax3.plot(t, dtheta_d[:,0], 'g', linewidth=2, label='Elbow (link1)')
	ax3.plot(t, dtheta_d[:,1],'b', linewidth=2, label='End-effector (link2)')
	
	# Now add the legend with some customizations.
	legend = ax3.legend(loc='upper right', shadow=True)
	ax3.set_title('Velocity of Arm at Waypoints')
	ax3.set_xlabel('Waypoint')
	ax3.set_ylabel('Velocity')
	
	plt.show()