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

%% Input Interface for motion configuration
dims = [1, 40]; % Dimension/size of the interface
% Header for input field - prompt to enter the direction 
prompt = {'Enter direction (forward, backward):'};
dlgtitle = 'Direction';     % Title of the interface
definput = {'forward'};     % Default input value
opts.Interpreter = 'tex';   % Define Interpreter for Latex formatting
answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
% Set flag for the corresponding direction (forward/backward motion)
if strcmp(answer,'backward') == 1
    direction = -1;  % backward motion
else 
    direction = 1;   % forward motion
end

%% Input interface for vehicle configuration 

% Initial/Start Pose configuration
title_start = 'Start Pose'; 
% Header for single input fields - prompt to enter the configuration parameters
prompt_start = {'x-coordinate','y-coordinate','\theta_0 [in degree]','\theta_1 [in degree]','\phi [in degree]','uncertainty in x','uncertainty in y','travel time'}; 
definput_start = {'0','0','0','0','0','random','random','10'};     % Default input values
input_start = inputdlg(prompt_start, title_start, dims, definput_start, opts); 
input_start = str2double(input_start); 

% Initial/Start Pose configuration
title_final = 'Final Pose'; 
% Header for single input fields - prompt to enter the configuration parameters
prompt_final = {'x-coordinate','y-coordinate','\theta_0 [in degree]','\theta_1 [in degree]','\phi [in degree]'};
definput_final = {'30','30','0','0','0'};   % Default input values
input_final = inputdlg(prompt_final, title_final, dims, definput_final, opts); 
input_final = str2double(input_final); 

%% Declaration and Initialization of variables

% Distance between front and rear axle of the truck
d0 = 2; 

% Distance between truck's rear axle and trailer's axle
d1 = 3; 

% Define and initialize variables 
% Initial/start pose
start.x1 = input_start(1);    % X-Coordinate of Trailer's rear axle
start.y1 = input_start(2);    % Y-Coordinate of Trailer's rear axle
theta0_s_d = input_start(3);  % Orientation angle of Truck in degree
start.theta0 = theta0_s_d*2*pi/360;
theta1_s_d = input_start(4); % Orientation angle of Trailer in degree
start.theta1 = theta1_s_d*2*pi/360;
phi_s_d = input_start(5);     % Steering angle of Truck in degree
start.phi = phi_s_d*2*pi/360;  
start.x0 = start.x1 + d1*cos(start.theta1); % X-Coordinate of Truck's rear axle
start.y0 = start.y1 + d1*sin(start.theta1); % Y-Coordinate of Truck's rear axle

uncertainty_x = input_start(6); % Deviation from predefined x-coordinate
uncertainty_y = input_start(7); % Deviation from predefined y-coordinate

% Input is not a number indicating chosen random uncertainty
if isnan(input_start(6))
    uncertainty_x = PositionUncertainty();
end
if isnan(input_start(7))
    uncertainty_y = PositionUncertainty();
end

% Final/end pose
final.x1 = input_final(1);   % X-Coordinate of Trailer's rear axle
final.y1 = input_final(2);   % Y-Coordinate of Trailer's rear axle
theta0_e_d = input_final(3);  % Orientation angle of Truck in degree
final.theta0 = theta0_e_d*2*pi/360;
theta1_e_d = input_final(4);  % Orientation angle of Trailer in degree
final.theta1 = theta1_e_d*2*pi/360;
phi_e_d = input_final(5);     % Steering angle of Truck in degree
final.phi = phi_e_d*2*pi/360; 
final.x0 = final.x1 + d1*cos(final.theta1); % X-Coordinate of Truck's rear axle
final.y0 = final.y1 + d1*sin(final.theta1); % Y-Coordinate of Truck's rear axle

% Build the vectors for initial and final pose of the vehicle
state_x0 = [start.x1, start.y1, start.theta0, start.theta1, start.phi];
state_x1 = [final.x1, final.y1, final.theta0, final.theta1, final.phi];

% Path Planning -> get polynomial coefficients for reference trajectory
coef = PathPlanner(state_x0, state_x1, d0, d1);

% Define travel time with interface input - predefined with 10s
T = input_start(8); 

% Weighting factors for state feedback based on Hurwitz criterion
% Assumption: All zeros of the characteristical polynomial = 2 (strictly positive)
k0 = 0.0625; 
k1 = 0.5; 
k2 = 0.75;  
k3 = 2; 

% change sign of k1 & k3 in case of backward motion
if direction == -1           
    k1 = k1 * -1; 
    k3 = k3 * -1;
end

% Define a set of parameters containing 
Parameters.coef=coef;             % Polynomial coeffients
Parameters.d0 = d0;               % Axle distances (Truck)
Parameters.d1 = d1;               % Axle distances (hitch length)
Parameters.T = T;                 % Travel time
Parameters.x0 = start.x1;         % Start pose of Truck/Trailer System
Parameters.x1 = final.x1;         % Final pose of Truck/Trailer System
Parameters.k0 = k0;               % Weighting factors (state feedback) 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 
Parameters.direction = direction; % Indication of motion direction 

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

% Solve ODE with varying start state depending on motion direction and uncertainty
if direction == 1           % forward motion
    odeStartState = [start.x0-uncertainty_x, start.y0-uncertainty_y, start.theta0, start.theta1, start.phi,...
                     xi_1, xi_2, xi_3];
elseif direction == -1      % backward motion 
    odeStartState = [final.x0-uncertainty_x, final.y0-uncertainty_y, final.theta0, final.theta1, final.phi,...
                 xi_1, xi_2, xi_3];
else                        % error case
    disp('ERROR') 
end

% Central function referencing to the other m-files via ODEFunc
[t, State] = ode45(@ODEFunc, [0,T], odeStartState, [], Parameters); 

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
axis([start.x0-5 final.x0+10 start.y0-5 final.y0+10]) % Scaling axis
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
