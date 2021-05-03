%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Exercise 2 Simulation 
%   Dynamic Feedforward Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear former data
clear
close
clc

% test scenarios
% 1 1 0 0 | 100 -30 0 0
% 1 1 0 0 | 100 100 0 0
% 1 1 45 0 | 100 100 0 0
% 1 1 45 0 | 100 100 90 0

% distance between front and rear axis
l_0 = 2; 

% Define and initialize variables
x0 = 1; 
y0 = 1; 
theta0_d = 0;
theta0 = theta0_d*2*pi/360;
phi0_d = 0; 
phi0 = phi0_d*2*pi/360; 
v0 = 1;

x1 = 100; 
y1 = -30;  
theta1_d = 0;
theta1 = theta1_d*2*pi/360;
phi1_d = 0; 
phi1 = phi1_d*2*pi/360; 
v1 = 1;

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0, y0, theta0, phi0, v0];
state_x1 = [x1, y1, theta1, phi1, v1];

% Function call => get polynomial coefficients
coef = PathPlanner1(state_x0,state_x1,l_0)

% Predefine time span T
T = 20;
t = linspace(1,100,1000);

Parameters.coef=coef; 
Parameters.l_0 = l_0; 
Parameters.T = T; 
Parameters.x0 = x0; 
Parameters.x1 = x1; 

[t,State] = ode45(@ODEFunc, [0,T], state_x0); 
%{
[t,y]=ode45(@vdp1,[0 20],[2 0]);   
          plot(t,y(:,1));
      solves the system y' = vdp1(t,y), using the default relative error
      tolerance 1e-3 and the default absolute tolerance of 1e-6 for each
      component, and plots the first component of the solution.
%}

