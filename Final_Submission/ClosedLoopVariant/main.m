%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   Examination Project
%   Truck/Trailer System
%
%   Closed Loop variant
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear former data
clear
close
clc

% Test scenarios
% 1 1 0 0 | 100 -30 0 0
% 1 1 0 0 | 100 100 0 0
% 1 1 45 0 | 100 100 0 0
% 1 1 45 0 | 100 100 90 0

% Distance between front and rear axle of the truck
d0 = 2; 

% Distance between truck's rear axle and trailer's axle
d1 = 3; 

% Define and initialize variables 
% Initial/start pose
start.x0 = 0;    % X-Coordinate of Truck's rear axle
start.y0 = 0;    % Y-Coordinate of Truck's rear axle
theta0_s_d = 0;  % Orientation angle of Truck in degree
start.theta0 = theta0_s_d*2*pi/360;
theta1_s_d = 0; % Orientation angle of Trailer in degree
start.theta1 = theta1_s_d*2*pi/360;
phi_s_d = 15;     % Steering angle of Truck in degree
start.phi = phi_s_d*2*pi/360;  

% Final/end pose
final.x0 = 100;   % X-Coordinate of Truck's rear axle
final.y0 = 30;   % Y-Coordinate of Truck's rear axle
theta0_e_d = 0;  % Orientation angle of Truck in degree
final.theta0 = theta0_e_d*2*pi/360;
theta1_e_d = 0;  % Orientation angle of Trailer in degree
final.theta1 = theta1_e_d*2*pi/360;
phi_e_d = 35;     % Steering angle of Truck in degree
final.phi = phi_e_d*2*pi/360; 

% Build the vectors for initial and final pose of the vehicle
state_x0 = [start.x0, start.y0, start.theta0, start.theta1, start.phi];
state_x1 = [final.x0, final.y0, final.theta0, final.theta1, final.phi];

% Path Planning -> get polynomial coefficients for reference trajectory
coef = PathPlanner(state_x0, state_x1, d0, d1);

% Predefine time span T - Travel Time
T = 5;

% Weighting factors for state feedback (random init) 
% Control gains -> only needed for closed loop
% Have to be at least >0 (Hurwitz polynomial 2nd order) for bigger order it
% To be determined
k0 = 0.00001; 
k1 = 1000; 
k2 = 1000; 
k3 = 1; 

% Define a set of parameters containing
% - polynomial coeffients
% - axle distances (vehicle)
% - travel time
% - start and final pose of Truck/Trailer System
% - weighting factors (state feedback) 
Parameters.coef=coef; 
Parameters.d0 = d0; 
Parameters.d1 = d1; 
Parameters.T = T; 
Parameters.x0 = start.x0;   
Parameters.x1 = final.x0;   
Parameters.k0 = k0; 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 

% Reference trajectory for plotting tests 
% Auxiliary graph for comparison with the resulting trajectory
i=0;
for t=0:0.01:T
    i=i+1;
    [Ref(i), ~, ~] = CalcRefValues(t, Parameters);
    xRef(i) = Ref(i).xRef;
    yRef(i) = Ref(i).yRef;
end

% Steady controller states
xi_1 = 1;
xi_2 = 0;
xi_3 = 0;

% Solve ODE
odeStartState = [start.x0, start.y0, start.theta0, start.theta1, start.phi,...
                 xi_1, xi_2, xi_3];
             
% Central function referencing to the other m-files via ODEFunc
[t, State] = ode45(@ODEFunc, [0,T], odeStartState, [], Parameters); 

% Determine the Trailer's position based on the Truck's coordinates
for i=1:length(t)
    x1(i) = State(i,1) - d1*cos(State(i,4));
    y1(i) = State(i,2) - d1*sin(State(i,4));
end

%% VISUALIZATION

% Moving Truck/Trailer visualization
figure(1)
MovingPlot(State, start, final, T, Parameters);
axis equal
grid on

%%
%{
% Plot resulting and reference trajectory (check overlapping)
figure(2)
plot(State(:,1), State(:,2), 'Color', [0.5 0 0]); % Resulting trajectory of Truck/Trailer System (truck perspective)
hold on
plot(x1,y1, 'Color', [0 0 0.5]); % Resulting trajectory of Truck/Trailer System (Trailer perspective)
hold on
plot(xRef, yRef, 'Color', [0 0.5 0]); % Reference trajectory of Truck/Trailer System
xlabel("x")
ylabel("y")
title("Trajectory of Truck/Trailer Model between Start and Final Pose")
%legend('Resulting Trajectory', 'Reference Trajectory', 'Location', 'northeastoutside')
legend('Resulting Trajectory Truck', 'Resulting Trajectory Trailer', 'Reference Trajectory', 'Location', 'northeastoutside')
axis equal
grid on 

xt = State(:,1);
yt = State(:,2); 

% First painting attempt
figure(3) 
plot(xt, yt, 'Color', [0.5 0 0]); % Resulting trajectory of Truck/Trailer System (truck perspective)
hold on
plot(x1, y1, 'Color', [0.5 0 0]); % Resulting trajectory of Truck/Trailer System (trailer perspective)
hold on

%% Start position
%{
% Define rectangles (shape of truck)
shape0 = polyshape([xt(1)-0.5 xt(1)-0.5 xt(1)+3.5 xt(1)+3.5], [yt(1)+1 yt(1)-1 yt(1)-1 yt(1)+1]);
plot(shape0)
hold on
plot([xt(1) xt(1)+3], [yt(1) yt(1)], 'Color', 'k', 'LineWidth', 2); 

plot([xt(1) xt(1)], [yt(1)-1 yt(1)+1], 'Color', 'k', 'LineWidth', 2); % Rear Axle
plot([xt(1)+3 xt(1)+3], [yt(1)-1 yt(1)+1], 'Color', 'k', 'LineWidth', 2); % Front Axle

plot([xt(1)-0.4 xt(1)+0.4], [yt(1)-1 yt(1)-1], 'Color', 'k', 'LineWidth', 4); % Right Rear Tyre
plot([xt(1)-0.4 xt(1)+0.4], [yt(1)+1 yt(1)+1], 'Color', 'k', 'LineWidth', 4); % Left Rear Tyre
plot([xt(1)+2.6 xt(1)+3.4], [yt(1)-1 yt(1)-1], 'Color', 'k', 'LineWidth', 4); % Right Front Tyre
plot([xt(1)+2.6 xt(1)+3.4], [yt(1)+1 yt(1)+1], 'Color', 'k', 'LineWidth', 4); % Left Front Tyre

% Define rectangles (shape of trailer)
shape0 = polyshape([x1(1)-0.5 x1(1)-0.5 x1(1)+1.5 x1(1)+1.5], [y1(1)+1 y1(1)-1 y1(1)-1 y1(1)+1]);
plot(shape0)
hold on
plot([x1(1) xt(1)], [y1(1) yt(1)], 'Color', 'k', 'LineWidth', 2); 

plot([x1(1) x1(1)], [y1(1)-1 y1(1)+1], 'Color', 'k', 'LineWidth', 2); % Rear Axle

plot([x1(1)-0.4 x1(1)+0.4], [y1(1)-1 y1(1)-1], 'Color', 'k', 'LineWidth', 4); % Right Rear Tyre
plot([x1(1)-0.4 x1(1)+0.4], [y1(1)+1 y1(1)+1], 'Color', 'k', 'LineWidth', 4); % Left Rear Tyre

axis equal
grid on
%}
refPointTruck = [xt(5500), yt(5500)];
refPointTrailer = [x1(5500), y1(5500)];
PlotState(refPointTruck, refPointTrailer, State)

axis equal
grid on
%}

