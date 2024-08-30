clc
clear
close all

% Simulation Config
dt = 1/50;
timeSimulation = 30;  

% ComPort Config
comPort = 'COM10';
baudRate = 115200;
timeOut = 10;
% Stop Flag
stopSimulationFlag = 0;


RollAngleLimit = 20*pi/180; % rad
PitchAngleLimit = 20*pi/180; % rad
VerticalVelocityLimit  = -2; % m/s
YawRateLimit  = 25*pi/180; % rad/s

