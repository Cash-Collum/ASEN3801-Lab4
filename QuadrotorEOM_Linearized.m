function var_dot = QuadrotorEOM_Linearized(t,var,g,m,I,deltaFC,deltaGC)

% States
del_x = var(:,1); % Devitaion from trim for Inertial X Position (m)
del_y = var(:,2); % Devitaion from trim for Inertial Y Position (m)
del_z = var(:, 3); % Devitaion from trim for Inertial Z Position (m)
del_psi = var(:,6); % Devitaion from trim for Body Yaw angle (rad)
del_theta = var(:,5); % Devitaion from trim for Body Pitch Angle (rad)
del_phi = var(:,4); % Devitaion from trim for Body Roll Angle (rad)
del_u_e = var(:, 7); % Devitaion from trim for X velcocity in Body Coordinates (m/s)
del_v_e = var(:, 8); % Devitaion from trim for Y velocity in Body Coordinates (m/s)
del_w_e = var(:, 9); % Devitaion from trim for Z velocity in Body Coordinates (m/s)
del_p = var(:,10); % Devitaion from trim for Body Roll rate (rad/s)
del_q = var(:, 11); % Devitaion from trim for Body Pitch rate (rad/s)
del_r = var(:, 12); % Devitaion from trim for Body Yaw rate (rad/s)

% Controls
del_Zc = deltaFC(3);
del_Lc = deltaGC(1);
del_Mc = deltaGC(2);
del_Nc = deltaGC(3);

var_dot = zeros(12,1); % Initializing state derivative vector



% Longitudinal Dynamics
var_dot(1) = del_u_e;
var_dot(7) = -g * del_theta;
var_dot(5) = del_q;
var_dot(11) = (1./I(2)) * del_Mc;
% Lateral Dynamics
var_dot(2) = del_v_e;
var_dot(8) = g * del_phi;
var_dot(4) = del_p;
var_dot(10) = (1/I(1)) * del_Lc;
%Vertical Dynamics
var_dot(3) = del_w_e;
var_dot(9) = (1/m) * del_Zc;
% Yaw Dynamics
var_dot(6) = del_r;
var_dot(12) = (1/I(3)) * del_Nc;

end