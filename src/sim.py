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

# --------------------------------------------------#
# Runs 2D simulation of a 2-link arm in a plane 
# Add forces to arm by clicking on the plot near 
# the joint you want to affect. 
#--------------------------------------------------#

class Arm():
	# Constants
	l1 = 0 			# link 1 length 
	l2 = 0 			# link 2 length
	m1 = 0 			# link 1 mass 
	m2 = 0 			# link 2 mass

	#r1 = 1.0 		# distance from first joint to center of mass for link 1
	#r2 = 1.0 	 	# distance from second joint to center of mass for link 2

	# Init Conditions
	theta = np.zeros((2,1)) 		# [shoulder theta, joint theta]
	dtheta = np.zeros((2,1))		# [shoulder angular vel, joint angular vel] 
	
	tau = np.zeros((2,1))			# [torque on link1 (shoulder), torque on link2 (elbow)]
	
	dt = 0.01 	# sim time step

	# constants in computation of inertia and coriolis forces matrices  
	alpha = 0
	beta = 0
	delta = 0
	
	# compute x,y position of arm's end-effector and elbow
	ee = np.zeros((2,1)) 
	elbow = np.zeros((2,1)) 

	# inertial matrix
	M = np.zeros((2,2))
	# coriolis forces matrix
	C = np.zeros((2,2))

	def __init__(self):
		# set up links and masses
		self.l1 = 1.0
		self.l2 = 1.0
		self.m1 = 0.5
		self.m2 = 0.5

		# set starting configuration
		self.theta[0] = pi/8
		self.theta[1] = pi/8

		# set (x,y) ee and elbow position 
		self.ee[0] = self.l1*cos(self.theta[0])
		self.ee[1] = self.l1*sin(self.theta[0])
		self.elbow[0] = self.ee[0] + self.l2*cos(self.theta[0] + self.theta[1])
		self.elbow[1] = self.ee[1] + self.l2*sin(self.theta[0] + self.theta[1])

		# compute constants 
		self.alpha = self.m1*self.l1**2 + self.m2*(self.l1**2 + self.l2**2)
		self.beta = self.m2*self.l1*self.l2
		self.delta = self.m2*self.l2**2

		# update M and C matrices given theta, dtheta information
		self.M = compute_M(self.alpha, self.beta, self.delta, self.theta)
		self.C = compute_C(self.alpha, self.beta, self.delta, self.theta, self.dtheta)

	# get forward kinematics of end-effector (end of link2)
	def fwd_kin_ee(self):
		x2 = self.l1*cos(self.theta[0]) + self.l2*cos(self.theta[0] + self.theta[1])
		y2 = self.l1*sin(self.theta[0]) + self.l2*sin(self.theta[0] + self.theta[1])
		return np.array([[x2],[y2]])

	# get forward kinematics of elbow elbow (end of link 1)
	def fwd_kin_elbow(self):
		x1 = self.l1*cos(self.theta[0])
		y1 = self.l1*sin(self.theta[0])
		return np.array([[x1], [y1]])

	# get inverse kinematics of end-effector to compute angles
	# needed in order to get to ee to (x,y) pos
	def inv_kin_ee(self, pos):
		x = pos[0]
		y = pos[1]

		num = x**2+y**2-self.l1**2-self.l2**2
		denom = 2*self.l1*self.l2
		s = 1-(num/denom)**2

		try: 
			b = sqrt(s)
		except:
			print "\n--------------------------------"
			print "WARNING: TRAJECTORY POINT ", (x,y) 
			print "OUTSIDE OF ROBOT'S WORKSPACE"
			print "--------------------------------\n"
			return None

		a = num/denom

		t1 = atan2(b, a)
		#t1_prime = atan2(-b, a)		
		t2 = atan2(y, x) - atan2(self.l2*sin(self.theta[1]), self.l1+self.l2*cos(self.theta[1]))

		config = np.zeros((2,1))
		config[0] = t1
		config[1] = t2

		return config

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

def plot_arm(fig, arm, traj):

	plt.clf()
	ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
	ax.grid()
	
	plt.plot([0, arm.x1], [0, arm.y1], 'b', linewidth=5)
	plt.plot([arm.x1, arm.x2], [arm.y1, arm.y2], 'b', linewidth=5)
	
	# plot joints
	plt.plot(0,0,'ko')
	plt.plot(arm.x1, arm.y1, 'ko') 
	plt.plot(arm.x2, arm.y2, 'ko')
	
	# plot trajectory waypoints
	plt.plot(traj[0],traj[1],'ro')

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

if __name__ == "__main__":

	# initialize default arm 
	arm = Arm()

	M = compute_M(arm.alpha, arm.beta, arm.delta, arm.theta)
	print "M: ", M 
	C = compute_C(arm.alpha, arm.beta, arm.delta, arm.theta, arm.dtheta)
	print "C: ", C

	arm.tau = np.array([[-1.6],[1.8]])
	ddtheta = fwd_dynamics(M, C, arm.tau, arm.dtheta)
	print "ddtheta: ", ddtheta
	tau_verified = inv_dynamics(M, C, arm.dtheta, ddtheta)
	print "tau_verified: ", tau_verified

	fig = plt.figure(figsize=(10,10))
	plt.clf()
	ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
	ax.grid()
	plt.hold(True)
	
	# start/end for trajectory
	arm_pos = arm.fwd_kin_ee()
	SX = arm_pos[0]; SY = arm_pos[1]
	GX = -1.0; GY = 1.5
	num_waypoints = 10
	
	# get trajectory in world coordinates (x,y) for plotting
	xy_traj = get_line_traj((SX,SY), (GX,GY), num_waypoints)
	print "xy_traj: ", xy_traj 

	# convert trajectory from world to cspace with inverse kinematics
	theta_d = traj_to_cspace(xy_traj, arm)
	print "configuration traj: ", theta_d

	dtheta_d = compute_dtheta(theta_d)
	ddtheta_d = compute_ddtheta(theta_d)

	print "dtheta_d: ", dtheta_d
	print "ddtheta_d: ", ddtheta_d
	
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

		plt.pause(1)

	#plt.show()
	
