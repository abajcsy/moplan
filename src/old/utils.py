import numpy as np
import matplotlib.pyplot as plt
import sys

import numpy.linalg
from numpy.linalg import *

import math
from math import  *

#-------------------------------------------------#
#------------ MOTION MATRICES   -------------#
#-------------------------------------------------#

# updates inertial forces due to acceleration of joints 
# M(theta) matrix based on current state information
def compute_M(alpha, beta, delta, theta):
	c2 = cos(theta[1])

	M = np.zeros((2,2))
	M[0][0] = alpha + 2*beta*c2
	M[0][1] = delta + beta*c2
	M[1][0] = delta + beta*c2
	M[1][1] = delta

	return M 

# updates coriolis and centrifugal forces C(theta, dtheta) matrix 
# based on current state information
def compute_C(alpha, beta, delta, theta, dtheta):
	s2 = sin(theta[1])
	
	C = np.zeros((2,2))
	C[0][0] = -beta*s2*dtheta[0] 
	C[0][1] = -beta*s2*(dtheta[0] + dtheta[1])
	C[1][0] = beta*s2*dtheta[0] 
	C[1][1] = 0.0

	return C

#---------------------------------------#
#------------ DYNAMICS --------------#
#---------------------------------------#
	
# returns ddtheta given torque M, C, tau, dtheta
def fwd_dynamics(M, C, tau, dtheta):
	ddtheta = np.dot(inv(M), tau - np.dot(C,dtheta))
	return ddtheta

# returns torque tau given M, C, theta, dtheta, ddtheta
def inv_dynamics(M, C, dtheta, ddtheta):
	tau = np.dot(M,ddtheta) + np.dot(C, dtheta)
	return tau

# given list of thetas, compute dtheta for each timestep
def compute_dtheta(theta_list, delta_t):
	(m,n) = theta_list.shape
	dtheta = np.zeros((m,n))
	dtheta[0][0] = 0 	#TODO is this right for all cases?
	dtheta[0][1] = 0

	for i in range(1,m):
		dtheta[i][0] = (theta_list[i][0] - theta_list[i-1][0])/delta_t
		dtheta[i][1] = (theta_list[i][1] - theta_list[i-1][1])/delta_t
	return dtheta		

# given list of thetas, computes ddtheta for each timestep
def compute_ddtheta(theta_list, delta_t):
	(m,n) = theta_list.shape
	ddtheta = np.zeros((m,n))
	ddtheta[0][0] = theta_list[1][0] 	#TODO is this right for all cases?
	ddtheta[0][1] = theta_list[1][1]

	for i in range(1,m-1):
		ddtheta[i][0] = (theta_list[i+1][0] - 2*theta_list[i][0] + theta_list[i-1][0])/delta_t
		ddtheta[i][1] = (theta_list[i+1][1] - 2*theta_list[i][0] + theta_list[i-1][1])/delta_t
	return ddtheta		
	
#---------------------------------------#
#----------- TRAJECTORIES --------------#
#---------------------------------------#

def get_line_traj(xy_start, xy_goal, num_waypoints):
	traj = np.zeros((num_waypoints,2))
	traj[0][0] = xy_start[0]
	traj[0][1] = xy_start[1]
	dist_x = xy_goal[0]-xy_start[0]
	dist_y = xy_goal[1]-xy_start[1]

	for i in range(1,num_waypoints):
		traj[i][0] = traj[i-1][0] + dist_x/(num_waypoints-1)
		traj[i][1] = traj[i-1][1] + dist_y/(num_waypoints-1)
		
	return traj

# convert trajectory of (x,y) points into (theta1, theta2) configurations
def traj_to_cspace(traj, arm):
	(m,n) = traj.shape
	q_traj = np.zeros((m,n))

	for i in range(m):
		print "traj[",i,"]: ", traj[i,:]
		q = arm.inv_kin_ee(traj[i,:])
		if q == None:
			print "\nINVALID/UNREACHABLE TRAJECTORY\n"
			return 
		q_traj[i][0] = q[0]
		q_traj[i][1] = q[1]

	return q_traj
	
def traj_tau_d(arm, theta_d, dtheta_d, ddtheta_d, num_waypoints):
	tau_d = np.zeros((num_waypoints, 2))
	for t in range(num_waypoints):
		theta_t_d = np.array([[theta_d[t][0]], [theta_d[t][1]]])
		dtheta_t_d = np.array([[dtheta_d[t][0]], [dtheta_d[t][1]]])
		ddtheta_t_d = np.array([[ddtheta_d[t][0]], [ddtheta_d[t][1]]])

		# compute new M and C matrices given theta_d
		M_t_d = compute_M(arm.alpha, arm.beta, arm.delta, theta_t_d)
		C_t_d = compute_C(arm.alpha, arm.beta, arm.delta, theta_t_d, dtheta_t_d)

		# compute torque for each waypoint along trajectory 
		tau_t_d = inv_dynamics(M_t_d, C_t_d, dtheta_t_d, ddtheta_t_d)
		tau_d[t][0] = tau_t_d[0]
		tau_d[t][1] = tau_t_d[1]

	print "tau_d:", tau_d
	return tau_d
	
def traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints):
	ddtheta_d_verified = np.zeros((num_waypoints, 2))
	for t in range(num_waypoints):
		print tau_d[t][1]
		theta_t_d = np.array([[theta_d[t][0]], [theta_d[t][1]]])
		dtheta_t_d = np.array([[dtheta_d[t][0]], [dtheta_d[t][1]]])
		ddtheta_t_d = np.array([[ddtheta_d[t][0]], [ddtheta_d[t][1]]])
		tau_t_d = np.array([[tau_d[t][0]], [tau_d[t][1]]])

		# compute new M and C matrices given theta_d
		M_t_d = compute_M(arm.alpha, arm.beta, arm.delta, theta_t_d)
		C_t_d = compute_C(arm.alpha, arm.beta, arm.delta, theta_t_d, dtheta_t_d)

		ddtheta_t_d = fwd_dynamics(M_t_d, C_t_d, tau_t_d, dtheta_t_d)
		ddtheta_d_verified[t][0] = ddtheta_t_d[0]
		ddtheta_d_verified[t][1] = ddtheta_t_d[1]

	print "ddtheta_d_verified:", ddtheta_d_verified
	return ddtheta_d_verified

#---------------------------------------#
#------ HUMAN INTERACTION  ------#
#---------------------------------------#	


def compute_tau_H(arm, tau_a, tau_d, theta_a, dtheta_a, ddtheta_a):
	M = compute_M(arm.alpha, arm.beta, arm.delta, theta_a)
	C = compute_C(arm.alpha, arm.beta, arm.delta, theta_a, dtheta_a)
	
	tau = inv_dynamics(M, C, dtheta_a, ddtheta_a)
	tau_H = tau - tau_a
	# or can I just do tau_actual - tau_demo to get the tau_human?
	return tau_H
	
#---------------------------------------#
#------------ PLOTTING --------------#
#---------------------------------------#

def tau_plot(plt, tau_d, t):
	fig2 = plt.figure(figsize=(15, 8))
	ax2 = fig2.add_subplot(111)
	ax2.grid()
	ax2.plot(t, tau_d[:,0], 'g', linewidth=2, label='Tau1')
	ax2.plot(t, tau_d[:,1],'b', linewidth=2, label='Tau2')
	
	# add the legend with some customizations.
	legend = ax2.legend(loc='upper right', shadow=True)
	ax2.set_title('Torque Applied at Waypoints')
	ax2.set_xlabel('Waypoint')
	ax2.set_ylabel('Torque')
	
def vel_plot(plt, dtheta_d, dtheta_actual, t):
	fig3 = plt.figure(figsize=(15, 8))
	ax3 = fig3.add_subplot(111)
	ax3.grid()
	ax3.plot(t, dtheta_d[:,0], 'g', linewidth=2, label='Computed Elbow (link1)')
	ax3.plot(t, dtheta_d[:,1],'b', linewidth=2, label='Computed End-effector (link2)')
	ax3.plot(t, dtheta_actual[:,0], 'r', linewidth=2, label='Actual Elbow (link1)')
	ax3.plot(t, dtheta_actual[:,1],'k', linewidth=2, label='Actual End-effector (link2)')
	
	# add the legend with some customizations.
	legend = ax3.legend(loc='upper right', shadow=True)
	ax3.set_title('Velocity of Arm at Waypoints')
	ax3.set_xlabel('Waypoint')
	ax3.set_ylabel('Velocity')
	
def state_plot(plt, theta_d, theta_actual, t):
	fig3 = plt.figure(figsize=(15, 8))
	ax3 = fig3.add_subplot(111)
	ax3.grid()
	ax3.plot(t, theta_d[:,0], 'g', linewidth=2, label='Computed Elbow (link1)')
	ax3.plot(t, theta_d[:,1],'b', linewidth=2, label='Computed End-effector (link2)')
	ax3.plot(t, theta_actual[:,0], 'r', linewidth=2, label='Actual Elbow (link1)')
	ax3.plot(t, theta_actual[:,1],'k', linewidth=2, label='Actual End-effector (link2)')
	
	# add the legend with some customizations.
	legend = ax3.legend(loc='upper right', shadow=True)
	ax3.set_title('Position of Arm at Waypoints')
	ax3.set_xlabel('Waypoint')
	ax3.set_ylabel('Position')
