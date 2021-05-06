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
% initial pose
x0 = 1; 
y0 = 1; 
theta0_d = 0;
theta0 = theta0_d*2*pi/360;
phi0_d = 0; 
phi0 = phi0_d*2*pi/360; 
v0 = 1;

% final pose
x1 = 10; 
y1 = 10;  
theta1_d = 0;
theta1 = theta1_d*2*pi/360;
phi1_d = 0; 
phi1 = phi1_d*2*pi/360; 
v1 = 1;

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0, y0, theta0, phi0, v0];
state_x1 = [x1, y1, theta1, phi1, v1];

% Function call => get polynomial coefficients for reference trajectory
coef = PathPlanner1(state_x0,state_x1,l_0);

% Predefine time span T - Travel Time
T = 5;

Parameters.coef=coef; 
Parameters.l_0 = l_0; 
Parameters.T = T; 
Parameters.x0 = x0; 
Parameters.x1 = x1; 

% solve ODE
%[t,State] = ode45(@ODEFunc, [0,T], state_x0(1:3),[], Parameters); 
[t,State] = ode45(@ODEFunc, [0,T], [x0, y0, theta0],[], Parameters); 

% return result to console (test)
State;

% plot resulting trajectory of the vehicle
figure(1)
plot(t, State(:,1)); 
xlabel("t")
ylabel("x")

figure(2)
plot(t, State(:,2)); 
xlabel("t")
ylabel("y")


figure(3)
plot(State(:,1), State(:,2)); 
xlabel("x")
ylabel("y")
