function [traj_progress, final_cost] = CHOMP(obs_cost, grad_x, grad_y, traj, iter, lambda, other_weight)

%step = 0.01;
%lambda = 0.1;
step = 0.3;
if nargin<6
    lambda = 0.8;
    other_weight = 1;
end

traj_progress = [traj];

%make K matrix
N=size(traj,2);
K=zeros(N-1,N-2);
for i=1:N-2
    K(i,i) = 1;
    K(i+1,i) = -1;
end
A=K'*K;
Aiv = inv(A);

for it = 1:iter
    %compute smoothness gradient
    smoothness_grad = zeros(2,N);
    for i=2:N-1
        smoothness_grad(:,i) = 2*traj(:,i)-traj(:,i-1)-traj(:,i+1);
    end
    smoothness_grad = smoothness_grad(:,2:N-1);
    
    
    %compute obstacle gradient
    obstacle_grad = zeros(2,N);
    for i=2:N-1
        %get gradient in obs_cost
        obstacle_grad(1,i) = grad_x(round(traj(1,i)),round(traj(2,i)));
        obstacle_grad(2,i) = grad_y(round(traj(1,i)),round(traj(2,i)));
    end
    obstacle_grad = obstacle_grad(:,2:N-1);
    
    
    %do the update
    grad = other_weight*smoothness_grad + lambda*obstacle_grad;
%     figure(42)
%     plot(it,ComputeCost(traj,obs_cost,lambda,other_weight),'x')
%     hold on
    %grad = obstacle_grad;
    inc_x = Aiv * grad(1,:)';
    inc_y = Aiv * grad(2,:)';

    
    traj(:,2:N-1) = traj(:,2:N-1) - step*[inc_x';inc_y'];
    
    traj_progress = [traj_progress;traj];
        
    
end

final_cost = ComputeCost(traj,obs_cost,lambda,other_weight);
end
