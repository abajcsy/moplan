close all
%chomp params
traj_points = 30;
lambda = 0.2;
other_weight = 0.1;
iter = 150;

epsilon = 60;

%world params
N = 151;
SX = 10;
SY = 10;
GX = 90;
GY = 90;
OBST = [50,60;100,70];

% N=351;
% SX=216;
% SY=103;
% GX=195;
% GY=213;
% OBST=[177,213];


world = zeros(N);
for i=1:size(OBST,1)
    world(OBST(i,1),OBST(i,2)) = 1; %point obstacles
end

obs_cost = bwdist(world);
obs_cost(obs_cost>epsilon)=epsilon;
obs_cost = 1/(2*epsilon)*(obs_cost-epsilon).^2;
figure(1)
imagesc(obs_cost')
grad_x = diff(obs_cost,1,1);
grad_y = diff(obs_cost,1,2);
hold on


%make straight line trajectory
traj = MakeStraightLineTraj(SX,SY,GX,GY,traj_points);
figure(1)
plot(traj(1,:),traj(2,:),'k');

[traj_progress, cost] = CHOMP(obs_cost, grad_x, grad_y, traj, iter, lambda,other_weight);


figure(1)
for i=1:iter
    plot(traj_progress(2*i+1,:),traj_progress(2*i+2,:),'g');
end