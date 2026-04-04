clear;
clc;
close all;
set(0,'DefaultFigureVisible','off');

%% Initialzing Data

% Constants

d = .06; % Radial distance to each motor (m)
km = .0024; % Motor constant N*m/(N)
% Control input array declaration
Z_c = [-1, -1, -1, -1];
L_c = [-d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2)];
M_c = [d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2)];
N_c = [-km, km, -km, km];
m = 0.068; %Kg
I_x = 5.8E-5; %Kg-M^2
I_y = 7.2E-5; %Kg-m^2
I_z = 1.0E-4; %Kg_m^2
I = [I_x;I_y;I_z];
nu = 1E-3; %N/(m/s)^2
mu = 2E-6; %N*m/(rad/s)^2
g = 9.81; % m / s^2


%% Task 1 
%------------------ Part 2 ------------------%
% Trim state hover
f1 = m * g / 4;
f2 = f1;
f3 = f2;
f4 = f3;
motor_forces_1 = [f1;f2;f3;f4];
control_input_array_trim = [Z_c; L_c; M_c; N_c] * motor_forces_1;
span_t = [0,10];
var_0 = zeros(12,1);
%initialize figure number and color arrays
fig_1_2=[121,122,123,124,125,126]; % Figure naming vector for task 1 part 2 (1_2)
col = {'r', 'b', 'g', 'k'}; % Color vector
%Change EOM_nodrag to QuadrotorEOM when accounting for drag
[t_nodrag,var_nodrag] = ode45(@(t,var)QuadrotorEOM_nodrag(t,var,g,m,I,d,km,nu,mu,motor_forces_1),span_t,var_0);
control_input_array_1_2_plot = control_input_array_trim * ones(1,length(t_nodrag));
PlotAircraftSim(t_nodrag,var_nodrag,control_input_array_1_2_plot,fig_1_2,col,'No Drag');

%------------------ Part 3 ------------------%
% Drag forces added
% Note that we are using the same initial conditions as before so nothing
% needs to change there. We created a function that calculates the
% equations of motion without accounting for drag and one that does account
% for drag (QuadrotorEOM) so we will use the ladder function.

% Figure numbers for this section
fig_1_3=[131,132,133,134,135,136];
[t_drag,var_drag] = ode45(@(t,var)QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces_1),span_t,var_0);
control_input_array_1_3_plot = control_input_array_trim * ones(1,length(t_drag));
PlotAircraftSim(t_drag,var_drag,control_input_array_1_3_plot,fig_1_3,col,'Drag'); % Note the control input array is the same for part 2 and 3

% In order to show that the trim state remains the same we compare the
% inertial velocities and the angular rates chaning would result in an
% altered state.

% To do this we will calculate the difference between the drag and no drag
% simulations for these specified variables and plot them if the state is
% not altered the difference should be zero for all 6 state variables in
% question.
% This is able to be done beacause the amount of iterations is the same for
% both simulations
% Interial velocities difference
delta_V = var_nodrag(:,7:9) - var_drag(:,7:9); 
% Angular Rates difference
delta_w = var_nodrag(:,10:12) - var_drag(:,10:12);

% ----------Plotting the differences----------
% Figure 3: Inertial velocity in body frame (u_e, v_e, w_e)
figure; 
set(gcf,'Name','\deltaV Between Drag and No Drag','NumberTitle','off');
subplot(3,1,1)
plot(t_drag,delta_V(:,1),'r','LineWidth',1.2); grid on
ylabel('\deltau (m/s)')
title('\deltaV Between Drag and No Drag')
subplot(3,1,2)
plot(t_drag,delta_V(:,2),'r','LineWidth',1.2); grid on
ylabel('\deltav (m/s)')
subplot(3,1,3)
plot(t_drag,delta_V(:,3),'r','LineWidth',1.2); grid on
ylabel('\deltaw (m/s)')
xlabel('Time (s)')
exportgraphics(gcf,'Delta_V.png','Resolution',300);

% Figure 4: Angular rates (p, q, r)
figure; 
set(gcf,'Name','\delta\omega Between Drag and No Drag','NumberTitle','off');
subplot(3,1,1)
plot(t_drag,delta_w(:,1),'r','LineWidth',1.2); grid on
ylabel('\deltap (rad/s)')
title('\delta\omega Between Drag and No Drag')
subplot(3,1,2)
plot(t_drag,delta_w(:,2),'r','LineWidth',1.2); grid on
ylabel('\deltaq (rad/s)')
subplot(3,1,3)
plot(t_drag,delta_w(:,3),'r','LineWidth',1.2); grid on
ylabel('\deltar (rad/s)')
xlabel('Time (s)')
exportgraphics(gcf,'Delta_w.png','Resolution',300);




%% ---------- Task 2 ---------- %%

% Part 3
% I already made this figure set so just going to modify it for part 3
fig_sets_25 = { [2511 2521 2531 2541 2551 2561]; % Part a
[2512 2522 2532 2542 2552 2562];% Part b 
[ 2513 2523 2533 2543 2553 2563];% Part c
[ 2514 2524 2534 2544 2554 2564];% Part d
[ 2515 2525 2535 2545 2555 2565];% Part e
[ 2516 2526 2536 2546 2556 2566];% Part f
};
fig_sets_23 = cell(size(fig_sets_25));

% Modifiying fig_sets to be for part 3 now 
for i = 1:length(fig_sets_25)
    fig_sets_23{i} = fig_sets_25{i} - 300;
end

% Deviations in trim
var_0 = zeros(12,1);


dev_angle = 5; % Degrees
dev_rate_angle = 0.1; % rad/s

dev_angle = deg2rad(dev_angle); % Radians

% New color vector for Linearized Dynamics
col_lin = {'b','r','g','m'};

% Initialize delta_gc and delta_fc whice are zeros here since there are no
% contriol inputs 
delta_fc = zeros(3,1);
delta_gc = delta_fc;
% Apply deviations to trim state 
for i = 1 : length(fig_sets_23)

del_var = zeros(12,1);

    if i == 1
        del_var(4,1) =  dev_angle; % Roll angle
    elseif i == 2
        del_var(5,1) =  dev_angle; % Pitch angle
    elseif i == 3
        del_var(6,1) =  dev_angle; % Yaw angle
    elseif i == 4
        del_var(10,1) = dev_rate_angle; % roll rate
    elseif i == 5
        del_var(11,1) =  dev_rate_angle; % Pitch rate
    elseif i == 6 
        del_var(12,1) = dev_rate_angle; % Yaw rate


    end
    del_var = del_var + var_0;
% Now running through ode45 and plotaircraft sim for non-linearized
[t_2_2_non_lin{i},var_2_2_non_lin{i}] = ode45(@(t,var)QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces_1),span_t,del_var);
% Backing out control input arrays
control_input_array_2_2_non_lin{i} = GetControlInputArray(t_2_2_non_lin{i},var_2_2_non_lin{i},m,g);


% Running now for linearized 
[t_2_2_lin{i},var_2_2_lin{i}] = ode45(@(t,var)QuadrotorEOM_Linearized(t,var,g,m,I,delta_fc,delta_gc),span_t,del_var);
% Backing out control input arrays
control_input_array_2_2_lin{i} = GetControlInputArray(t_2_2_lin{i},var_2_2_lin{i},m,g);
% Plot the results for Part 3
PlotAircraftSim(t_2_2_non_lin{i}, var_2_2_non_lin{i}, control_input_array_2_2_non_lin{i}, fig_sets_23{i}, col,'Non-Linearized');
PlotAircraftSim(t_2_2_lin{i}, var_2_2_lin{i}, control_input_array_2_2_lin{i}, fig_sets_23{i}, col_lin,'Linearized');








end

% part 5



% Apply deviations to trim state 
for i = 4:6 % CASES D-F
    del_var = zeros(12,1);
    if i == 1
        del_var(4,1) =  dev_angle; % Roll angle
    elseif i == 2
        del_var(5,1) =  dev_angle; % Pitch angle
    elseif i == 3
        del_var(6,1) =  dev_angle; % Yaw angle
    elseif i == 4
        del_var(10,1) = dev_rate_angle; % roll rate
    elseif i == 5
        del_var(11,1) =  dev_rate_angle; % Pitch rate
    elseif i == 6 
        del_var(12,1) = dev_rate_angle; % Yaw rate


    end

% Now running through ode45 and plotaircraft sim
[t_2_5{i},var_2_5{i}] = ode45(@(t,var)QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu),span_t,del_var);
% Backing out control input arrays
control_input_array_2_5{i} = GetControlInputArray(t_2_5{i},var_2_5{i},m,g);
% Plot the results for Part 5
PlotAircraftSim(t_2_5{i}, var_2_5{i}, control_input_array_2_5{i}, fig_sets_25{i}, col,'Controlled');
% Plot the results for Part 5
PlotAircraftSim(t_2_2_non_lin{i}, var_2_2_non_lin{i}, control_input_array_2_2_non_lin{i}, fig_sets_25{i}, col_lin, 'Non-Controlled');
end






% Function
function PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col,label)
% Added label input for legend entries in overlayed plots

% State Variables
X = aircraft_state_array(:,1); % Inertial X Position (m)
Y = aircraft_state_array(:,2); % Inertial Y Position (m)
Z = aircraft_state_array(:,3); % Inertial Z Position (m)
psi = aircraft_state_array(:,6); % Body Yaw angle (rad)
theta = aircraft_state_array(:,5); % Body Pitch Angle (rad)
phi = aircraft_state_array(:,4); % Body Roll Angle (rad)
u_e = aircraft_state_array(:,7); % X velocity in Body Coordinates (m/s)
v_e = aircraft_state_array(:,8); % Y velocity in Body Coordinates (m/s)
w_e = aircraft_state_array(:,9); % Z velocity in Body Coordinates (m/s)
p = aircraft_state_array(:,10); % Body Roll rate (rad/s)
q = aircraft_state_array(:,11); % Body Pitch rate (rad/s)
r = aircraft_state_array(:,12); % Body Yaw rate (rad/s)

% Control Inputs
Z_c = control_input_array(1,:); % Body Z Force (N)
L_c = control_input_array(2,:); % Body Roll Moment (Nm)
M_c = control_input_array(3,:); % Body Pitch Moment (Nm)
N_c = control_input_array(4,:); % Body Yaw Moment (Nm)

% Make sure time is a column for plotting against n x 1 state vectors
time = time(:);

%% Figure 1: Inertial Position (X, Y, Z)
figure(fig(1));
set(gcf,'Name','Inertial Position','NumberTitle','off');

subplot(3,1,1)
plot(time,X,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('X (m)')
title('Inertial Position')
legend('show','Location','best')

subplot(3,1,2)
plot(time,Y,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('Y (m)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,Z,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('Z (m)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(1)), 'Resolution', 300);

%% Figure 2: Euler Angles (psi, theta, phi)
figure(fig(2));
set(gcf,'Name','Euler Angles','NumberTitle','off');

subplot(3,1,1)
plot(time,psi,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\psi (rad)')
title('Euler Angles')
legend('show','Location','best')

subplot(3,1,2)
plot(time,theta,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\theta (rad)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,phi,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\phi (rad)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(2)), 'Resolution', 300);

%% Figure 3: Inertial velocity in body frame (u_e, v_e, w_e)
figure(fig(3));
set(gcf,'Name','Body-frame Velocities','NumberTitle','off');

subplot(3,1,1)
plot(time,u_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('u (m/s)')
title('Inertial Velocity in Body Frame')
legend('show','Location','best')

subplot(3,1,2)
plot(time,v_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('v (m/s)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,w_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('w (m/s)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(3)), 'Resolution', 300);

%% Figure 4: Angular rates (p, q, r)
figure(fig(4));
set(gcf,'Name','Angular Rates','NumberTitle','off');

subplot(3,1,1)
plot(time,p,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('p (rad/s)')
title('Angular Velocity')
legend('show','Location','best')

subplot(3,1,2)
plot(time,q,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('q (rad/s)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,r,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('r (rad/s)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(4)), 'Resolution', 300);

%% Figure 5: Control inputs (Z_c, L_c, M_c, N_c)
figure(fig(5));
set(gcf,'Name','Control Inputs','NumberTitle','off');

subplot(4,1,1)
plot(time,Z_c(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('Z_c (N)')
title('Control Inputs')
legend('show','Location','best')

subplot(4,1,2)
plot(time,L_c(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('L_c (Nm)')
legend('show','Location','best')

subplot(4,1,3)
plot(time,M_c(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('M_c (Nm)')
legend('show','Location','best')

subplot(4,1,4)
plot(time,N_c(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('N_c (Nm)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(5)), 'Resolution', 300);

%% Figure 6: 3D trajectory (with positive height upward)
figure(fig(6));
set(gcf,'Name','3D Trajectory','NumberTitle','off');

plot3(X,Y,-Z,col{4},'LineWidth',1.2,'DisplayName',label); hold on; grid on

% Mark start and end without putting them in the legend
plot3(X(1),Y(1),-Z(1),'o','Color',col{3},'MarkerSize',8,'LineWidth',2, ...
    'HandleVisibility','off');
plot3(X(end),Y(end),-Z(end),'x','Color',col{1},'MarkerSize',8,'LineWidth',2, ...
    'HandleVisibility','off');

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Height (m)')
title('3D Aircraft Path (positive up)')
legend('show','Location','best')
view(3)
axis equal
% Enforce minimum span of 20 on each axis
min_span = 20;
xl = xlim; yl = ylim; zl = zlim;
if diff(xl) < min_span
    mid = mean(xl);
    xlim([mid - min_span/2, mid + min_span/2]);
end
if diff(yl) < min_span
    mid = mean(yl);
    ylim([mid - min_span/2, mid + min_span/2]);
end
if diff(zl) < min_span
    mid = mean(zl);
    zlim([mid - min_span/2, mid + min_span/2]);
end
exportgraphics(gcf, sprintf('fig%d.png', fig(6)), 'Resolution', 300);

end