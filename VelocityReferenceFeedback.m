function [Fc, Gc] = VelocityReferenceFeedback(t, var)

m = 0.068;
g = 9.81;

K1 = .0014;   % from 3.1
K2 = .0026;
K3 = .0035;   % from 3.5

% --- states ---
phi = var(:,7);
theta = var(:,8);
p = var(:,10);
q = var(:,11);
r = var(:,12);

u = var(:,4);
v = var(:,5);

% reference velocity
if t <= 2
    u_r = 0.5;
    v_r = 0;   % or swap for lateral case
else
    u_r = 0;
    v_r = 0;
end

% outer loop
theta_c = -K3 * (u - u_r);
phi_c   =  K3 * (v_r - v);

% inner loop
L_c = -K1*(phi - phi_c) - K2*p;
M_c = -K1*(theta - theta_c) - K2*q;
N_c = -0.004*r;

Fc = [0; 0; m*g];
Gc = [L_c; M_c; N_c];


end
