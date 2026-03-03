%% Initialzing Data
load("/Users/parkerhimes/Documents/MATLAB/ASEN_3801/Lab_4/RSdata_nocontrol.mat");
aircraft_state_array = rt_estim.signals.values;
time = rt_estim.time;
control_input_array = 
%% Function
% function PlotAircraftSim (time,aircraft_state_array,control_input_array,fig,col)
% 
% 
% % Organizing data
% X = aircraft_state_array(:,1); % Inertial X Position (m)
% Y = aircraft_state_array(:,2); % Inertial Y Position (m)
% Z = aircraft_state_array(:,3); % Inertial Z Position (m)
% psi = aircraft_state_array(:,4); % Body Yaw angle (rad)
% theta = aircraft_state_array(:,5); % Body Roll Angle (rad)
% phi = aircraft_state_array(:,6); % Body Pitch Angle (rad)
% u_e = aircraft_state_array(:,7); % X velcocity in Body Coordinates (m/s)
% v_e = aircraft_state_array(:,8); % Y velocity in Body Coordinates (m/s)
% w_e = aircraft_state_array(:,9); % Z velocity in Body Coordinates (m/s)
% p = aircraft_state_array(:,10); % Body Roll rate (rad/s)
% q = aircraft_state_array(:,11); % Body Pitch rate (rad/s)
% r = aircraft_state_array(:,12); % Body Yaw rate (rad/s)
% 
% 
