import numpy as np
import matplotlib.pyplot as plt
import pylab
import matplotlib.image as mpimg

from numpy import *
from scipy.ndimage.morphology import distance_transform_edt
from chomp import chomp

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
	
def run_planner(num_traj_points, lambd, other_weight, iter, epsilon, N, SX, SY, GX, GY, OBST, world):
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

if __name__ == '__main__':
	#setup the world and relevant parameters
	num_traj_points = 30
	
	lambd = 0.2
	other_weight = 0.1
	
	iter = 150	
	epsilon = 60

	# world params
	N = 151
	SX = 10
	SY = 10
	GX = 140
	GY = 140
	# define pt-based obstacles 
	OBST = np.array([[50,60],[51,60],[50,61],[51,61],[100,70]])
	world = np.zeros((N,N))
	
	# do initial trajectory planning
	run_planner(num_traj_points, lambd, other_weight, iter, epsilon, N, SX, SY, GX, GY, OBST, world)
	
	# start to execute trajectory
	
	# at time t, get an interferance/force from human at location (x,y)
	# add new obstacle at point where person interfered
	human_obs = np.array([75,65])
	OBST = np.vstack((OBST, human_obs))
	SX = 40
	SY = 75
	
	# replan trajectory from the point of disturbance, given now input
	run_planner(num_traj_points, lambd, other_weight, iter, epsilon, N, SX, SY, GX, GY, OBST, world)
	
	