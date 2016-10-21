import numpy as np
import matplotlib.pyplot as plt
import pylab
import matplotlib.image as mpimg

from numpy import *
from scipy.ndimage.morphology import distance_transform_edt

# set default weights and lambda if none passed in
def chomp(obs_cost, grad_x, grad_y, traj, iter, lambd=0.8, other_weight=1):
	# step = 0.01
	# lambd = 0.1
	step = 0.3
	
	#if nargin < 6:
	#	lambd = 0.8
	#	other_weight = 1

	traj_progress = np.array(traj)
	print 'traj'
	print traj
	print 'traj_progress'
	print traj_progress

	# make K matrix
	N = size(traj, 1)
	K = np.zeros((N-1,N-2))
	print np.shape(K)
	
	for i in range(N-2):
		K[i][i] = 1
		K[i+1][i] = -1
	
	A = np.dot(K.transpose(), K)
	Aiv = np.linalg.inv(A)
	print np.shape(Aiv)

	for it in range(iter):
		# compute smoothness gradient
		smoothness_grad = np.zeros((2,N))
		for i in range(1,N-1):
			smoothness_grad[:,i] = 2*traj[:,i]-traj[:,i-1]-traj[:,i+1]

		smoothness_grad = smoothness_grad[:,1:N-1]
		
		# compute obstacle gradient
		obstacle_grad = np.zeros((2,N))
		
		print 'grad_x'
		print grad_x
		for i in range(1,N-1):
			# get gradient in obs_cost
			obstacle_grad[0][i] = grad_x[int(round(traj[0][i]))][int(round(traj[1][i]))]
			obstacle_grad[1][i] = grad_y[int(round(traj[0][i]))][int(round(traj[1][i]))]
		
		obstacle_grad = obstacle_grad[:,1:N-1]
		print np.shape(obstacle_grad)
	
		# do the update
		grad = other_weight*smoothness_grad + lambd*obstacle_grad;
		print np.shape(grad)
	
	    # figure(42)
		# plot(it,ComputeCost(traj,obs_cost,lambda,other_weight),'x')
		# hold on
		# grad = obstacle_grad
		inc_x = np.dot(Aiv, grad[0,:].transpose())
		print np.shape(inc_x)
	
		inc_y = np.dot(Aiv, grad[1,:].transpose())
		print np.shape(inc_x)
		
		print "inc_x"
		#print inc_x
		print "inc_y"
		#print inc_y
		print 'traj[;,1:N-1]'
		#print traj[:,1:N-1]
		
		inc_xy = np.column_stack((inc_x, inc_y)).T #CHECK THIS!
		print np.shape(inc_xy)
		print np.shape(traj[:,1:N-1])
	
		#print size(traj[:,1:N-1])
		#print size(step*inc_xy)
		
		traj[:,1:N-1] = traj[:,1:N-1] - step*inc_xy
		
		traj_progress = np.concatenate([traj_progress,traj])
		
	#print traj_progress
	final_cost = compute_cost(traj, obs_cost, lambd, other_weight)
	
	return (traj_progress, final_cost)

def compute_cost(traj, obs_cost, lambd, other_weight):
	N = size(traj,1)

	# obstacle cost
	obs = 0
	for i in range(N):
		obs = obs + obs_cost[int(round(traj[0][i]))][int(round(traj[1][i]))]

	# smoothness cost
	smt = 0
	for i in range(1,N):
		smt = smt + np.linalg.norm(traj[:,i] - traj[:,i-1])**2
	
	smt = smt*0.5
	cost = other_weight*smt + lambd*obs
	
	return cost
	
# returns linear trajectory from (start_x,start_y) to (goal_x,goal_y)
def get_linear_traj(start_x, start_y, goal_x, goal_y, waypoints):
	traj = np.zeros((2,waypoints))
	traj[0][0] = start_x
	traj[1][0] = start_y
	dist_x = goal_x-start_x
	dist_y = goal_y-start_y

	for i in range(1,waypoints):
		traj[0][i]=traj[0][i-1]+dist_x/(waypoints-1)
		traj[1][i]=traj[1][i-1]+dist_y/(waypoints-1)
	
	return traj

if __name__ == '__main__':
	num_traj_points = 30
	
	lambd = 0.2
	other_weight = 0.1
	
	iter = 150	
	epsilon = 60

	# world params
	N = 151
	SX = 10
	SY = 10
	GX = 90
	GY = 90
	# define pt-based obstacles 
	OBST = np.array([[50,60],[100,70]])

	world = np.zeros((N,N))
	
	# set each pt obstacle into the world
	for obj in OBST:
		i = obj[0]
		j = obj[1]
		world[i][j] = 1 

	# calculate distance to the seeds
	obs_cost = distance_transform_edt(world == 0) # bwdist() in matlab	
	obs_cost[obs_cost > epsilon] = epsilon
	obs_cost = 1.0/(2.0*epsilon)*(obs_cost-epsilon)**2.0
	
	# note: 255-obs_cost is just to invert the colors for visualization 
	obs_cost_plt = plt.imshow(obs_cost) 
	plt.title('Distance Map')
	# plt.show()
	
	print obs_cost
	
	grad_x = np.diff(obs_cost,1,0)
	grad_y = np.diff(obs_cost,1,1)
	plt.hold(True)
	
	# make straight line trajectory
	traj = get_linear_traj(SX, SY, GX, GY, num_traj_points)
	# figure(1)
	plt.plot(traj[0,:],traj[1,:],'k')

	(traj_progress, cost) = chomp(obs_cost, grad_x, grad_y, traj, iter, lambd, other_weight)
	
	for i in range(iter):
		plt.plot(traj_progress[2*i+1,:], traj_progress[2*i+2,:],'g')
		
	plt.show()