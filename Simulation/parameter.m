% parameter.m
% Global variables for physical, velocity, and PID parameters
global b d r vR Kp Ki Kd kp1 ki1 kd1 kp2 ki2 kd2;

%% Physical parameters
b = 246;        % Axial distance between 2 driving wheels (mm)
d = 176;        % Distance from the center of the sensor array to the drive shaft (mm)
r = 40;         % Radius of each drive wheel (mm)

%% Velocity parameter
vR = 500;       % Base velocity (mm/s)

%% Line Tracking PID parameters
Kp = 0.045;
Ki = 0.00000;
Kd = 0.0001;

%% Left Motor PID parameters
kp1 = 1.6036; %1
ki1 = 23.765; %13
kd1 = 0.0088;

%% Right Motor PID parameters
kp2 = 1.5792;
ki2 = 26.1527;
kd2 = 0.0076;
