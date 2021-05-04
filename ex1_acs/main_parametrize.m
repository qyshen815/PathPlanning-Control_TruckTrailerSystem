%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Simple Path Planning 
%   main file (call function PathPlanner1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close
clc

% test scenarios
% 1 1 0 0 | 100 -30 0 0
% 1 1 0 0 | 100 100 0 0
% 1 1 45 0 | 100 100 0 0
% 1 1 45 0 | 100 100 90 0

% Define and initialize variables
x0 = 1; 
y0 = 1; 
theta0 = -90; 
phi0 = 0; 
v0 = 1;

x1 = 100; 
y1 = 50; 
theta1 = -45; 
phi1 = 0; 
v1 = 1;

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0, y0, theta0, phi0, v0];
state_x1 = [x1, y1, theta1, phi1, v1];

% Function call => get polynomial coefficients
coef = PathPlanner2(state_x0,state_x1)

%{
a = coef(1);
b = coef(2);
c = coef(3);
d = coef(4);
e = coef(5);
f = coef(6);

% Crate and plot the polynomial
minX = x0; 
maxX = x1;
length = 1000; 
x = linspace(minX,maxX,length);
x_til = linspace(minX-x0,maxX-x0,length);
fx = a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;


% Plot the path
figure(1)
plot(x, fx, 'LineWidth', 2.0);
xlabel('x in meters');
ylabel('y in meters');
title('Simple Path Planning');
grid on;
hold on;
%}