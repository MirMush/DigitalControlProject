%% Initialize and run Robot_model_P_Control_ZOH_2025b

clc;
clear;

mdl = 'Robot_model_P_Control_ZOH_2025b';

% Load the model
load_system(mdl);

%% Model parameters

L = 4.5e-3; % Henry
R = 4.29; % Ohm
Kt = 0.009; % Nm/A
Kb = Kt;   % V/(rad/s)
J = 1e-6;  % (Kg m^2) = Nm/(rad/s^2)
D = 5e-5;  % Nm/(rad/s)
gear = 9.68;
wheelRadius = 0.03; % (m)
r2m = wheelRadius/gear; % (radian/s to meter/s)
m = 0.95; % (kg)
Jr = m * r2m*r2m/2;
J = J + Jr; % to take into account the mass of the robot    
B   = 0.15;    
mesDelay = 10*1e-3;
Ts = 0.01;



%% Make variables available to Simulink
% Base workspace variables
assignin('base','r2m',r2m);
assignin('base','D',D);
assignin('base','B',B);
assignin('base','J',J);
assignin('base','mesDelay',mesDelay);
assignin('base','Kt',Kt);
assignin('base','L',L);
assignin('base','R',R);
assignin('base','Kb',Kb);
assignin('base','Ts',Ts);


% Set the proportional gain
Kp = 0.1202;
Ti = 0.0685;
assignin('base','Kp',Kp);

%% Simulation settings
set_param(mdl,'StopTime','10');

% Run the simulation
simOut = sim(mdl);

simOut.tout