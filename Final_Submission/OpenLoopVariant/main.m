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
% maybe define a start and end struct
% initial pose
x0_s = 0; 
y0_s = 0; 

theta0_s_d = 0;
theta0_s = theta0_s_d*2*pi/360;

theta1_s_d = 0;
theta1_s = theta1_s_d*2*pi/360;

phi_s_d = 0; 
phi_s = phi_s_d*2*pi/360; 

xi_1 = 1;
xi_2 = 1;
xi_3 = 1; 

% final pose
x0_e = 30; 
y0_e = 100;  
theta0_e_d = 0;
theta0_e = theta0_e_d*2*pi/360;
theta1_e_d = 0;
theta1_e = theta1_e_d*2*pi/360;
phi_e_d = 0; 
phi_e = phi_e_d*2*pi/360; 

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0_s, y0_s, theta0_s, theta1_s, phi_s];
state_x1 = [x0_e, y0_e, theta0_e, theta1_e, phi_e];

% Function call => get polynomial coefficients for reference trajectory
coef = PathPlanner(state_x0,state_x1,d0,d1);

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
Parameters.x0 = x0_s;   % here x0 is x-coordinate of start pose
Parameters.x1 = x0_e;   % here x1 is x-coordinate of end pose 
Parameters.k0 = k0; 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 

% reference trajectory for plotting 
i=0;
for t=0:0.01:T
    i=i+1;
    [Ref(i), ~, ~] = CalcRefValues(t, Parameters);
    xRef(i) = Ref(i).xRef;
    yRef(i) = Ref(i).yRef;
end

% solve ODE
% Y_State?
% Ausgänge evtl erweitern für Referenzwerte und Pose
odeStartState = [x0_s, y0_s, theta0_s, theta1_s, phi_s,  xi_1, xi_2, xi_3];

[t,State] = ode45(@ODEFunc, [0,T], odeStartState ,[], Parameters); 

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

