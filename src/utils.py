import numpy as np
import matplotlib.pyplot as plt
import sys

import numpy.linalg
from numpy.linalg import *

import math
from math import  *

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

# returns ddtheta given torque M, C, tau, dtheta
def fwd_dynamics(M, C, tau, dtheta):
	ddtheta = np.dot(inv(M), tau - np.dot(C,dtheta))
	return ddtheta

# returns torque tau given M, C, theta, dtheta, ddtheta
def inv_dynamics(M, C, dtheta, ddtheta):
	tau = np.dot(M,ddtheta) + np.dot(C, dtheta)
	return tau

def get_line_traj((SX,SY), (GX,GY), num_waypoints):
	traj = np.zeros((num_waypoints,2))
	traj[0][0] = SX
	traj[0][1] = SY
	dist_x = GX-SX
	dist_y = GY-SY

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

	
# given list of thetas, compute dtheta for each timestep
def compute_dtheta(theta_list):
	(m,n) = theta_list.shape
	dtheta = np.zeros((m,n))
	dtheta[0][0] = 0 	#TODO is this right for all cases?
	dtheta[0][1] = 0

	for i in range(1,m):
		dtheta[i][0] = theta_list[i][0] - theta_list[i-1][0]
		dtheta[i][1] = theta_list[i][1] - theta_list[i-1][1]
	return dtheta		

# given list of thetas, computes ddtheta for each timestep
def compute_ddtheta(theta_list):
	(m,n) = theta_list.shape
	ddtheta = np.zeros((m,n))
	ddtheta[0][0] = theta_list[1][0] 	#TODO is this right for all cases?
	ddtheta[0][1] = theta_list[1][1]

	for i in range(1,m-1):
		ddtheta[i][0] = theta_list[i+1][0] - 2*theta_list[i][0] + theta_list[i-1][0]
		ddtheta[i][1] = theta_list[i+1][1] - 2*theta_list[i][0] + theta_list[i-1][1]
	return ddtheta		