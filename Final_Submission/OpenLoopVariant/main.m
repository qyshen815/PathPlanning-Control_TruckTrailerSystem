%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Examination Project
%   Truck Trailor System
%
%   Open Loop variant for testing 
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

% distance between front and rear axle of the truck
d0 = 2; 

% distance between truck's rear axle and trailer's axle
d1 = 3; 

% Define and initialize variables
% initial pose
x0 = 1; 
y0 = 1; 

theta0_d = 0;
theta0 = theta0_d*2*pi/360;

theta1_d = 0;
theta1 = theta1_d*2*pi/360;

phi0_d = 0; 
phi0 = phi0_d*2*pi/360; 
%v0 = 1;

xi_1 = 1;
xi_2 = 1;
xi_3 = 1; 

% final pose
%x1 = 10; 
%y1 = 10;  
%theta1_d = 0;
%theta1 = theta1_d*2*pi/360;
%phi1_d = 0; 
%phi1 = phi1_d*2*pi/360; 
%v1 = 1;

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0, y0, theta0, theta1, phi0, xi_1, xi_2, xi_3];
%state_x1 = [x1, y1, theta1, phi1, v1];

% Function call => get polynomial coefficients for reference trajectory
coef = PathPlanner1(state_x0,state_x1,L);

% Predefine time span T - Travel Time
T = 5;

% weighting factors for state feedback (random init) 
% control gains --> only needed for closed loop
% have to be at least >0 (Hurwitz polynomial 2nd order) for bigger order it
% has to be determined
k0 = 1; 
k1 = 2; 
k2 = 3; 
k3 = 4; 

Parameters.coef=coef; 
Parameters.d0 = d0; 
Parameters.d1 = d1; 
Parameters.T = T; 
Parameters.x0 = x0; 
Parameters.x1 = x1; 
Parameters.k0 = k0; 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 

% test
% reference trajectory for plotting
i=0;
for t=0:0.01:T
    i=i+1;
    [xRef(i), ~, ~, yRef(i), ~, ~] = CalcRefValues(t, Parameters);
end


% solve ODE
% Y_State?
[t,State] = ode45(@ODEFunc, [0,T], state_x0,[], Parameters); 

% plot resulting trajectory of the vehicle
%{
figure(1)
plot(t, State(:,1)); 
xlabel("t")
ylabel("x")

figure(2)
plot(t, State(:,2)); 
xlabel("t")
ylabel("y")
%}

figure(3)
plot(State(:,1), State(:,2)); 
hold on
% plot xref, yref -> correct if maximum overlapping
plot(xRef, yRef); 
xlabel("x")
ylabel("y")
