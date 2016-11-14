import sim
import sys

#arg1 = sys.argv[1]
#arg2 = sys.argv[2]

# start/end for trajectory
xy_start = [1.5, -1.0]
xy_goal = [-1.0, 1.5]
num_waypoints = 20

sim.basic_sim(xy_start, xy_goal, num_waypoints)