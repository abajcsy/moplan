function cost = ComputeCost(traj, obs_cost, lambda, other_weight)
N=size(traj,2);

%obstacle cost
obs = 0;
for i=1:N
    obs = obs + obs_cost(round(traj(1,i)),round(traj(2,i)));
end

%smoothness cost
smt = 0;
for i=2:N
    smt = smt + norm(traj(:,i)-traj(:,i-1)).^2;
end
smt=smt*0.5;
cost = other_weight*smt + lambda*obs;
end