theta1 = pi/8;
theta2 = pi/8;
dtheta1 = 0;
dtheta2 = 0;
dtheta = [dtheta1; dtheta2]

s1 = sin(theta1);
s2 = sin(theta2);
c1 = cos(theta1);
c2 = cos(theta2);

m1 = 0.5;
m2 = 0.5;
l1 = 1;
l2 = 1;

alpha = m1*l1.^2 + m2*(l1.^2 + l2.^2)
beta = m2*l1*l2
gamma = m2*l2.^2

t1 = 1.5
t2 = -1.9
tau = [t1; t2]

M = [alpha+2*beta*c2 gamma+beta*c2 ; gamma+beta*c2 gamma]
C = [-beta*s2*dtheta2 -beta*s2*(dtheta1+dtheta2) ; beta*s2*dtheta1 0]

ddtheta = M^(-1)*(tau - C*dtheta)

tau_verified = M*ddtheta + C*dtheta