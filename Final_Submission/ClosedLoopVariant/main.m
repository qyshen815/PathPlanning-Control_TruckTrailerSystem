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

%% Input interface for vehicle configuration 

dims = [1, 35]; % Dimension/size of the interface
% Header for single input fields - prompt to enter the configuration parameters
prompt = {'x-coordinate','y-coordinate','theta0 [in degree]','theta1 [in degree]','phi [in degree]'}; 

% Initial/Start Pose configuration
title_start = 'Start Pose'; 
definput_start = {'0','0','0','0','0'};     % Default input values
input_start = inputdlg(prompt, title_start, dims, definput_start); 
input_start = str2double(input_start); 

% Initial/Start Pose configuration
title_final = 'Final Pose'; 
definput_final = {'30','30','0','0','0'};   % Default input values
input_final = inputdlg(prompt, title_final, dims, definput_final); 
input_final = str2double(input_final); 

%% Declaration and Initialization of variables

% Distance between front and rear axle of the truck
d0 = 2; 

% Distance between truck's rear axle and trailer's axle
d1 = 3; 

% Define and initialize variables 
% Initial/start pose
start.x1 = input_start(1);    % X-Coordinate of Truck's rear axle
start.y1 = input_start(2);    % Y-Coordinate of Truck's rear axle
theta0_s_d = input_start(3);  % Orientation angle of Truck in degree
start.theta0 = theta0_s_d*2*pi/360;
theta1_s_d = input_start(4); % Orientation angle of Trailer in degree
start.theta1 = theta1_s_d*2*pi/360;
phi_s_d = input_start(5);     % Steering angle of Truck in degree
start.phi = phi_s_d*2*pi/360;  
start.x0 = start.x1;
start.y0 = start.y1;

% Final/end pose
final.x1 = input_final(1);   % X-Coordinate of Truck's rear axle
final.y1 = input_final(2);   % Y-Coordinate of Truck's rear axle
theta0_e_d = input_final(3);  % Orientation angle of Truck in degree
final.theta0 = theta0_e_d*2*pi/360;
theta1_e_d = input_final(4);  % Orientation angle of Trailer in degree
final.theta1 = theta1_e_d*2*pi/360;
phi_e_d = input_final(5);     % Steering angle of Truck in degree
final.phi = phi_e_d*2*pi/360; 
final.y0 = final.y1; 
final.x0 = final.x1; 

% Build the vectors for initial and final pose of the vehicle
% trailer
state_x0 = [start.x1, start.y1, start.theta0, start.theta1, start.phi];
state_x1 = [final.x1, final.y1, final.theta0, final.theta1, final.phi];

% Path Planning -> get polynomial coefficients for reference trajectory
coef = PathPlanner(state_x0, state_x1, d0, d1);

% Predefine time span T - Travel Time
T = 10;

% Weighting factors for state feedback based on Hurwitz criterion
% Assumptions for control gains (precision vs performance): 
% - k0      --> 0
% - k1 & k2 --> inf
% - k3      --> 1
%{
k0 = 0; 
k1 = 0; 
k2 = 0; 
k3 = 0; 
%}

k0 = 0.0625; 
k1 = -0.75; %+-0.5
k2 = 0.75; % 0.75 
k3 = -2; %+-2


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
Parameters.x0 = start.x1;   
Parameters.x1 = final.x1;   
Parameters.k0 = k0; 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 

% Steady controller states
xi_1 = 1;
xi_2 = 0;
xi_3 = 0;

%% Trajectory calculation of stabilizing feedback control for Truck/Trailer system

% Reference trajectory for plotting tests 
% Auxiliary graph for comparison with the resulting trajectory
i=0;
for t=0:0.01:T
    i=i+1;
    [Ref(i), ~, ~] = CalcRefValues(t, Parameters);
    xRef(i) = Ref(i).xRef;
    yRef(i) = Ref(i).yRef;
end

% Solve ODE
%odeStartState = [start.x1+d1*cos(start.theta1), start.y1+d1*sin(start.theta1)-3, start.theta0, start.theta1, start.phi,...
                % xi_1, xi_2, xi_3];
             
odeStartState = [final.x1+d1*cos(start.theta1), final.y1+d1*sin(start.theta1)-3, final.theta0, final.theta1, final.phi,...
                 xi_1, xi_2, xi_3];
                     
% Central function referencing to the other m-files via ODEFunc
[t, State] = ode45(@ODEFunc, [0,T], odeStartState, [], Parameters); 

% Backward Motion
%w = eig(State)

% Determine the Trucks's position based on State vector
for i=1:length(t)
    x0(i) = State(i,1);
    y0(i) = State(i,2);
end

% Determine the Trailer's position based on the Truck's coordinates
for i=1:length(t)
    x1(i) = State(i,1) - d1*cos(State(i,4));
    y1(i) = State(i,2) - d1*sin(State(i,4));
end

%% Visualization 


% Plot 1: Forward Motion - Moving Truck/Trailer
figure(1)
set(gcf, 'Position', get(0, 'Screensize'));
MovingPlot(State, start, final, T, Parameters);


% Plot 2: Configuration View - Discrete Points of Truck/Trailer
figure(2)
p1 = 1; 
p2 = round(0.2*length(State));
p3 = round(0.5*length(State));
p4 = round(0.7*length(State));
p5 = length(State);
p = [p1, p2, p3, p4, p5]; % Discrete points of plotting Truck/Trailer visualization
for i = 1 : length(p)
    PlotState([x0(p(i)), y0(p(i))], [x1(p(i)), y1(p(i))], State, p(i))
end
hold on
plot(x0, y0, 'Color', 'r');  % Resulting trajectory of Truck/Trailer System (truck perspective)
hold on
plot(x1, y1, 'Color', 'b');  % Resulting trajectory of Truck/Trailer System (Trailer perspective)
title('Plot 2: Configuration View - Discrete Points of Truck/Trailer')
xlabel('x-coordinate')
ylabel('y-coordinate')
%axis([start.x0-5 final.x0+10 start.y0-5 final.y0+10]) % Scaling axis
grid on

% Plot 3: Reference Trajectory vs Resulting Trajectories (check overlapping)
figure(3) 
plot(x0, y0, 'Color', 'r'); % Resulting trajectory of Truck/Trailer System (Truck perspective)
hold on
plot(x1, y1, 'Color', 'b'); % Resulting trajectory of Truck/Trailer System (Trailer perspective)
hold on
plot(xRef, yRef, 'Color', [0 0.5 0]); % Reference trajectory of Truck/Trailer System
title('Plot 3: Reference Trajectory vs Resulting Trajectories')
xlabel('x-coordinate')
ylabel('y-coordinate')
legend('Resulting Trajectory Truck', 'Resulting Trajectory Trailer', 'Reference Trajectory', 'Location', 'northeastoutside')
axis equal
grid on


% Plot 4: Add on - Backward Motion
%figure(4) 
%title('Plot 4: Add on - Backward Motion')
%xlabel('x-coordinate')
%ylabel('y-coordinate')
%axis([start.x0-5 final.x0+10 start.y0-5 final.y0+10]) % Scaling axis
%grid on
%}
