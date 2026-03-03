function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)


X = var(:,1); % Inertial X Position (m)
Y = var(:,2); % Inertial Y Position (m)
Z = var(:, 3); % Inertial Z Position (m)
psi = var(:,4); % Body Yaw angle (rad)
theta = var(:,5); % Body Roll Angle (rad)
phi = var(:,6); % Body Pitch Angle (rad)
u_e = var(:, 7); % X velcocity in Body Coordinates (m/s)
v_e = var(:, 8); % Y velocity in Body Coordinates (m/s)
w_e = var(:, 9); % Z velocity in Body Coordinates (m/s)
p = var(:,10); % Body Roll rate (rad/s)
q = var(:, 11); % Body Pitch rate (rad/s)
r = var(:, 12); % Body Yaw rate (rad/s)

var = [X Y Z psi theta phi u_e v_e w_e p q r]';



f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

omega = [p; q; r];

g = [0,0, -9.8];


cpsi = cos(psi);  spsi = sin(psi);
cth  = cos(theta); sth  = sin(theta);
cphi = cos(phi);  sphi = sin(phi);

% derivitave velocity 
x_dot = [ cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi] ;
y_dot = [cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi];
z_dot = [-sth, sphi*cth,   cphi*cth ];

v_body = [u_e; v_e; w_e];

% Inertial position rates
pos_dot = [x_dot, y_dot, z_dot] * v_body;

xE_dot = pos_dot(1);
yE_dot = pos_dot(2);
zE_dot = pos_dot(3);


F_total = f1 + f2 + f3 + f4;


var_dot = [xE_dot yE_dot zE_dot psi_dot theta_dot phi_dot p_dot q_dot r_dot]';





Thrust = [0; 0; F_total];




































