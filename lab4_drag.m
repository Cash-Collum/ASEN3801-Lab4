%% Initialzing Data
load("RSdata_nocontrol.mat");
aircraft_state_array = rt_estim.signals.values;
time = rt_estim.time;
num_pts = length(time);
time_10s = linspace(0,10,num_pts);
control_input_array = rt_motor.signals.values;
fig = [1 2 3 4 5 6 7 8];             
col = {'b','ro','gx'};
PlotAircraftSim(time_10s,aircraft_state_array,control_input_array,fig,col)
% Function
function PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col)


% State Variables
X = aircraft_state_array(:,1); % Inertial X Position (m)
Y = aircraft_state_array(:,2); % Inertial Y Position (m)
Z = aircraft_state_array(:,3); % Inertial Z Position (m)
psi = aircraft_state_array(:,4); % Body Yaw angle (rad)
theta = aircraft_state_array(:,5); % Body Roll Angle (rad)
phi = aircraft_state_array(:,6); % Body Pitch Angle (rad)
u_e = aircraft_state_array(:,7); % X velcocity in Body Coordinates (m/s)
v_e = aircraft_state_array(:,8); % Y velocity in Body Coordinates (m/s)
w_e = aircraft_state_array(:,9); % Z velocity in Body Coordinates (m/s)
p = aircraft_state_array(:,10); % Body Roll rate (rad/s)
q = aircraft_state_array(:,11); % Body Pitch rate (rad/s)
r = aircraft_state_array(:,12); % Body Yaw rate (rad/s)

% Control Inputs
z_c = control_input_array(:,1); % Body Z Force (N)
L_c = control_input_array(:,2); % Body Roll Moment (Nm)
M_c = control_input_array(:,3); % Body Pitch Moment (Nm)
N_c = control_input_array(:,4); % Body Yaw Moment (Nm)

%Aerodynamic forces and moments
co_force=(1e-3); %kgm^2
co_moment=(2e-6);%Nm/(rad/s)^2
drag_x=-co_force*u_e.*abs(u_e);
drag_y=-co_force*v_e.*abs(v_e);
drag_z=-co_force*w_e.*abs(w_e);
m_aerox=-co_moment.*p.*abs(p);
m_aeroy=-co_moment.*q.*abs(q);
m_aeroz=-co_moment.*r.*abs(r);
%sum total forces and moments
total_fx=drag_x;
total_fy=drag_y;
total_fz=z_c+drag_z;
total_mx=m_aerox;
total_my=m_aeroy;
total_mz=m_aeroz;
 
% Figure 8: Total Drag Moments
figure(fig(8)); 
subplot(4,1,1)
plot(time,total_mx,col{1},'LineWidth',1.2); grid on
ylabel('Aero x moment')
title('Total Drag Moments')
subplot(4,1,2)
plot(time,total_my,col{1},'LineWidth',1.2); grid on
ylabel('Aero y moment')
subplot(4,1,3)
plot(time,total_mz,col{1},'LineWidth',1.2); grid on
ylabel('Aero z moment')

% Figure 3: Inertial velocity in body frame (u_e, v_e, w_e)
figure(fig(3)); 
set(gcf,'Name','Body-frame Velocities','NumberTitle','off');
subplot(3,1,1)
plot(time,u_e,col{1},'LineWidth',1.2); grid on
ylabel('u (m/s)')
title('Inertial Velocity in Body Frame')
subplot(3,1,2)
plot(time,v_e,col{1},'LineWidth',1.2); grid on
ylabel('v (m/s)')
subplot(3,1,3)
plot(time,w_e,col{1},'LineWidth',1.2); grid on
ylabel('w (m/s)')
xlabel('Time (s)')

% Figure 4: Angular rates (p, q, r)
figure(fig(4)); 
set(gcf,'Name','Angular Rates','NumberTitle','off');
subplot(3,1,1)
plot(time,p,col{1},'LineWidth',1.2); grid on
ylabel('p (rad/s)')
title('Angular Velocity')
subplot(3,1,2)
plot(time,q,col{1},'LineWidth',1.2); grid on
ylabel('q (rad/s)')
subplot(3,1,3)
plot(time,r,col{1},'LineWidth',1.2); grid on
ylabel('r (rad/s)')
xlabel('Time (s)')



end