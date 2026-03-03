%% Initialzing Data
load("/Users/parkerhimes/Documents/MATLAB/ASEN_3801/Lab_4/RSdata_nocontrol.mat");
aircraft_state_array = rt_estim.signals.values;
time = rt_estim.time;

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

% Plots

% Figure 1: Inertial Position (X, Y, Z)
figure(fig(1));
set(gcf,'Name','Inertial Position','NumberTitle','off');
subplot(3,1,1)
plot(time,X,col{1},'LineWidth',1.2); grid on
ylabel('X (m)')
title('Inertial Position')
subplot(3,1,2)
plot(time,Y,col{1},'LineWidth',1.2); grid on
ylabel('Y (m)')
subplot(3,1,3)
plot(time,Z,col{1},'LineWidth',1.2); grid on
ylabel('Z (m)')
xlabel('Time (s)')

% Figure 2: Euler Angles (psi, theta, phi)
figure(fig(2));
set(gcf,'Name','Euler Angles','NumberTitle','off');
subplot(3,1,1)
plot(time,psi,col{1},'LineWidth',1.2); grid on
ylabel('\psi (rad)')
title('Euler Angles')
subplot(3,1,2)
plot(time,theta,col{1},'LineWidth',1.2); grid on
ylabel('\theta (rad)')
subplot(3,1,3)
plot(time,phi,col{1},'LineWidth',1.2); grid on
ylabel('\phi (rad)')
xlabel('Time (s)')

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

% Figure 5: Control inputs (z_c, L_c, M_c, N_c)
figure(fig(5)); 
set(gcf,'Name','Control Inputs','NumberTitle','off');
subplot(4,1,1)
plot(time,z_c,col{1},'LineWidth',1.2); grid on
ylabel('z_c (N)')
title('Control Inputs')
subplot(4,1,2)
plot(time,L_c,col{1},'LineWidth',1.2); grid on
ylabel('L_c (Nm)')
subplot(4,1,3)
plot(time,M_c,col{1},'LineWidth',1.2); grid on
ylabel('M_c (Nm)')
subplot(4,1,4)
plot(time,N_c,col{1},'LineWidth',1.2); grid on
ylabel('N_c (Nm)')
xlabel('Time (s)')

% Figure 6: 3D trajectory (with positive height upward)
figure(fig(6)); 
set(gcf,'Name','3D Trajectory','NumberTitle','off');
plot3(X,Y,-Z,col{1},'LineWidth',1.2); hold on; grid on
% Mark start and end
plot3(X(1),Y(1),-Z(1),col{2});
plot3(X(end),Y(end),-Z(end),col{3});
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Height (m)')
title('3D Aircraft Path (positive up)')
view(3)
axis equal
hold off


end