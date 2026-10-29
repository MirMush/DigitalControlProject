%% 1. Initialize Workspace
clc;
clear;
close all;

mdl = 'Robot_model_Heading_Control_ZOH_2025b';

%% 2. Model & Controller Parameters
% Physical Robot & Motor Parameters
L = 4.5e-3;             % Henry
R = 4.29;               % Ohm
Kt = 0.009;             % Nm/A
Kb = Kt;                % V/(rad/s)
J = 1e-6;               % (Kg m^2) = Nm/(rad/s^2)
D = 5e-5;               % Nm/(rad/s)
gear = 9.68;
wheelRadius = 0.03;     % (m)
r2m = wheelRadius/gear; % (radian/s to meter/s)
m = 0.95;               % (kg)
Jr = m * r2m * r2m / 2;
J = J + Jr;             % Total inertia including robot mass
B = 0.15;               % Wheelbase (m)
mesDelay = 10e-3;       % Measurement delay (s)
Ts = 0.01;              % Sample time (s)

% Wheel Velocity PI Gains
Kp = 0.1202;
Ti = 0.0685;

% Turn Rate & Heading Gains
Kpt = 3;                % Turn rate P gain
taui_t = 0.4;           % Turn rate integral time [s]
Kph = 0.3;              % Placeholder value for heading P gain

% Input Sequences & Steps
vel_seq = [0 0 1]';
heading_seq= [ 1 1 1]';
turn_seq = [0 0 0]';
t_step = 1;

%% 3. Push Variables to Base Workspace (Required for Simulink & Linearize)
assignin('base', 'L', L);
assignin('base', 'R', R);
assignin('base', 'Kt', Kt);
assignin('base', 'Kb', Kb);
assignin('base', 'J', J);
assignin('base', 'D', D);
assignin('base', 'r2m', r2m);
assignin('base', 'B', B);
assignin('base', 'mesDelay', mesDelay);
assignin('base', 'Ts', Ts);
assignin('base', 'Kp', Kp);
assignin('base', 'Ti', Ti);
assignin('base', 'Kpt', Kpt);
assignin('base', 'taui_t', taui_t);
assignin('base', 'Kph', Kph);
assignin('base', 'vel_seq', vel_seq);
assignin('base', 'turn_seq', turn_seq);
assignin('base', 't_step', t_step);
assignin('base', 'heading_seq', heading_seq);
%% 4. Load System & Estimate Heading Transfer Function
load_system(mdl);
open_system(mdl);

% Define linearization I/O points
ios(1) = linio(strcat(mdl, '/Heading Controller/Uh'), 1, 'openinput');
ios(2) = linio(strcat(mdl, '/Robot model'), 3, 'openoutput');
setlinio(mdl, ios);

% Linearize the model at t = 1.0s
op = [1.0];
sys = linearize(mdl, ios, op);

% Extract Transfer Function & Plot Bode
[num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gheading = minreal(tf(num, den));

w = logspace(0, 2, 1000);
figure(220);
bode(Gheading, w);
grid on;
title('Heading Open-Loop Transfer Function (G_{heading})');

%% 5. Run Simulation
set_param(mdl, 'StopTime', '10');
out = sim(mdl);

%% 6. Plot Simulation Outputs
figure('Name', 'Robot Simulation Results', 'Position', [100, 100, 1000, 700]);

% 1. Left Output
subplot(2, 2, 1);
plot(out.left);
title('Left Output');
grid on;

% 2. Right Output
subplot(2, 2, 2);
if isprop(out, 'right')
    plot(out.right);
elseif isprop(out, 'out_right')
    plot(out.out_right);
else
    plot(out_right);
end
title('Right Output');
grid on;

% 3. Pose
subplot(2, 2, 3);
plot(out.pose);
title('Robot Pose');
grid on;

% 4. Robot State/Output
subplot(2, 2, 4);
plot(out.robot);
title('Robot State/Output');
grid on;