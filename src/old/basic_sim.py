

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
	dtheta_d = compute_dtheta(theta_d, num_waypoints)
	ddtheta_d = compute_ddtheta(theta_d, num_waypoints)
	
	# compute tau from the theta, dtheta, ddtheta
	tau_d = traj_tau_d(arm2, theta_d, dtheta_d, ddtheta_d, num_waypoints)
	# just sanity check the tau by computing ddtheta
	# ddtheta_d_verified = traj_ddtheta_d(arm, theta_d, dtheta_d, ddtheta_d, tau_d, num_waypoints)

	# initialize graphing 
	fig = plt.figure(figsize=(10,10))
	# store the velocities that are actually being applied at each waypoint
	dtheta_actual = np.zeros((num_waypoints,2))
	theta_actual = np.zeros((num_waypoints,2))
	for t in range(num_waypoints):	
		dt = 1.0/num_waypoints
		
		# store vel and position for plotting 
		dtheta_actual[t][0] = arm2.dtheta[0]
		dtheta_actual[t][1] = arm2.dtheta[1]
		
		theta_actual[t][0] = arm2.theta[0]
		theta_actual[t][1] = arm2.theta[1]
		
		M_t = compute_M(arm2.alpha, arm2.beta, arm2.delta, arm2.theta)
		C_t = compute_C(arm2.alpha, arm2.beta, arm2.delta, arm2.theta, arm2.dtheta)
		
		print "tau_d[t]:", tau_d[t].T
		print "arm.dtheta:", arm2.dtheta
		
		# set arm's current torque to the computed torque we need
		arm2.tau[0] = tau_d[t][0]
		arm2.tau[1] = tau_d[t][1]
		
		# compute accel with fwd dynamics given the torque and velocity 
		ddtheta_t = fwd_dynamics(M_t, C_t, arm2.tau, arm2.dtheta)
		
		print 'ddtheta_t: ', ddtheta_t
		
		# update dtheta and theta
		dtheta1_new = arm2.dtheta[0] + ddtheta_t[0]*dt
		dtheta2_new = arm2.dtheta[1] + ddtheta_t[1]*dt
		
		print "dtheta1_new: ", dtheta1_new
		print "dtheta2_new: ", dtheta2_new
		
		theta1_new = arm2.theta[0] + dtheta1_new*dt
		theta2_new = arm2.theta[1] + dtheta2_new*dt
		
		# update velocity and position 
		arm2.dtheta[0] = dtheta1_new
		arm2.dtheta[1] = dtheta2_new
		
		arm2.theta[0] = theta1_new
		arm2.theta[1] = theta2_new
		
		ee_pos = arm2.fwd_kin_ee()
		
		plt.clf()
		# plot the end-effector position 
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
		ax.grid()
		ax.text(.05, .05, 'TRAJ ee(x,y): ('+str(round(xy_traj[t][0],3))+','+str(round(xy_traj[t][1],3))+')', 
				horizontalalignment='left',
				verticalalignment='bottom',
				transform=ax.transAxes, 
				fontsize=15)
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
	
	t = [x for x in range(num_waypoints)]
	
	# plot the tau's over time
	tau_plot(plt, tau_d, t)
	
	# plot velocities over time 
	vel_plot(plt, dtheta_d, dtheta_actual, t)
	
	# plot state over time 
	state_plot(plt, theta_d, theta_actual, t)
	
	plt.show()
	

