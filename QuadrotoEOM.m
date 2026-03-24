function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)


x = var(1,1); % Inertial X Position (m)
y = var(2,1); % Inertial Y Position (m)
z = var(3,1); % Inertial Z Position (m)
psi = var(6,1); % Body Yaw angle (rad)
theta = var(5,1); % Body Pitch Angle (rad)
phi = var(4,1); % Body Roll Angle (rad)
u_e = var(7,1); % X velcocity in Body Coordinates (m/s)
v_e = var(8,1); % Y velocity in Body Coordinates (m/s)
w_e = var(9,1); % Z velocity in Body Coordinates (m/s)
p = var(10,1); % Body Roll rate (rad/s)
q = var(11,1); % Body Pitch rate (rad/s)
r = var(12,1); % Body Yaw rate (rad/s)





Ix = I(1,1);
Iy = I(2,1);
Iz = I(3,1);

Z_c = [-1, -1, -1, -1]*motor_forces;
L_c = [-d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2)] * motor_forces;
M_c = [d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2)] * motor_forces;
N_c = [km, -km, km, -km]*motor_forces;

f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

F_total = f1 + f2 + f3 + f4;




cpsi = cos(psi);  spsi = sin(psi);
cth  = cos(theta); sth  = sin(theta);
cphi = cos(phi);  sphi = sin(phi);

% derivitave velocity 
x_dot = [ cth.*cpsi,  sphi.*sth.*cpsi - cphi.*spsi,  cphi.*sth.*cpsi + sphi.*spsi ] ;
y_dot = [ cth.*spsi,  sphi.*sth.*spsi + cphi.*cpsi,  cphi.*sth.*spsi - sphi.*cpsi ] ;
z_dot = [ -sth, sphi.*cth,  cphi.*cth ] ;
v_body = [u_e; v_e; w_e];

% Inertial position rates
pos_dot = [x_dot; y_dot; z_dot] * v_body;

xE_dot = pos_dot(1);
yE_dot = pos_dot(2);
zE_dot = pos_dot(3);

% Euler Angle Rates
phi_dot = (p + (sin(phi) .* tan(theta) .* q) + (cos(phi) .* tan(theta) .* r));
theta_dot = ((cos(phi) .* q) + (-sin(phi) .* r));
psi_dot = ((sin(phi) .* sec(theta)) .* q + (cos(phi) .* sec(theta) .* r));

% Drag Moments
L = -mu * sqrt(p.^2 + q.^2 + r.^2) .* p;
M = -mu * sqrt(p.^2 + q.^2 + r.^2) .* q;
N = -mu * sqrt(p.^2 + q.^2 + r.^2) .* r;
% Drag Forces
X = -nu * sqrt(p.^2 + q.^2 + r.^2) .* u_e;
Y = -nu * sqrt(p.^2 + q.^2 + r.^2) .* v_e;
Z = -nu * sqrt(p.^2 + q.^2 + r.^2) .* w_e;

% Control Forces



% Velocity Rates
u_e_dot = ((r .* v_e - q .* w_e) + (g .* -sin(theta)) + (X ./ m));
v_e_dot = ((p .* w_e - r .* u_e) + (g .* cos(theta) .* sin(phi)) + (Y ./ m));
w_e_dot = ((q .* u_e - p .* v_e) + (g .* cos(theta) .* cos(phi)) + (Z ./ m)+(Z_c./m));



% Angular Velocity Rates
p_dot = (((Iy - Iz) ./ Ix .* q .* r) + (L ./ Ix) + L_c ./ Ix);
q_dot = (((Iz - Ix) ./ Iy .* p .* r) + (M ./ Iy) + M_c ./ Iy);
r_dot = (((Ix - Iy) ./ Iz .* p .* q) + (N ./ Iz) + N_c ./ Iz);



var_dot = [xE_dot yE_dot zE_dot psi_dot theta_dot phi_dot u_e_dot v_e_dot w_e_dot p_dot q_dot r_dot]';


Thrust = [0; 0; F_total];






































