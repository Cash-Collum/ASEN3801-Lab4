function [Fc, Gc] = InnerLoopFeedback(var)
m = 0.068;
g = 9.81;
k = 0.004;
Ix = 5.8E-5;

tau1 = .5;
tau2 = .05; 
lambda1 = -1/tau1;
lambda2 = -1/tau2;

% roll gains
kd_phi = -(lambda1 + lambda2) * Ix;
kp_phi =  (lambda1 * lambda2) * Ix;

% pitch gains
kd_theta = -(lambda1 + lambda2) * Iy;
kp_theta =  (lambda1 * lambda2) * Iy;

% control calcuations
phi = var(:,4);
theta = var(:,5);
p = var(:,10);
q = var(:,11);
r = var(:,12);
Z_c =  m* g;
L_c = -kp_phi * phi - kd_phi * p;
M_c = -kp_theta * theta - kd_theta * q;
N_c = -k * r;

Fc = [0,0,Z_c]';
Gc = [ L_c, M_c, N_c]';



end

