function PlotState(refPointTruck, refPointTrailer, State, k)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file creates a plot around one reference 
%   point and takes all the geometrical dimensions of the 
%   Truck/Trailer System into account. It implements the 
%   orientation and steering angles with regard to the axles 
%   and tyres of the vehicle.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract orientation and steering angles from struct
theta0 = State(k,3); 
theta1 = State(k,4); 
phi = State(k,5); 

%% Truck plotting 

% Main vehicle axle
SingleElementPlot(theta0, refPointTruck, 0, 0, 3, 0)

% Rear axle
SingleElementPlot(theta0, refPointTruck, 0, -1, 0, 1)

% Front axle
SingleElementPlot_Steering(theta0, phi, refPointTruck, 0, 1, 0, -1)

% Right rear tyre
SingleElementPlot(theta0, refPointTruck, -0.4, -1, 0.4, -1)

% Left rear tyre
SingleElementPlot(theta0, refPointTruck, -0.4, 1, 0.4, 1)

% Right front tyre
SingleElementPlot_Steering(theta0, phi, refPointTruck, -0.4, -1, 0.4, -1)

% Left front tyre
SingleElementPlot_Steering(theta0, phi, refPointTruck, -0.4, 1, 0.4, 1)

%% Truck shape plotting
x0 = refPointTruck(1); % Extracting coordinates of reference point
y0 = refPointTruck(2); 
shape0 = polyshape([x0-0.5 x0-0.5 x0+3.5 x0+3.5], [y0+1 y0-1 y0-1 y0+1]); % Normal Truck shape with geometrical sizes
shape0_rot = rotate(shape0, theta0/(2*pi)*360, refPointTruck); % Rotating Truck shape
plot(shape0_rot, 'FaceColor', 'r') % Plot rotating Truck shape
hold on


%% Trailer plotting

% Calculate difference between reference points of Truck and Trailer
deltaRefPoint = [refPointTruck(1)-refPointTrailer(1), refPointTruck(2)-refPointTrailer(2)];

% Main axle connecting to Truck
% Theta1 is not included due to a fixed connection to the Truck's reference point (center of gravitiy) 
SingleElementPlot(0, refPointTrailer, 0, 0, deltaRefPoint(1), deltaRefPoint(2))

% Rear axle
SingleElementPlot(theta1, refPointTrailer, 0, -1, 0, 1)

% Right rear tyre
SingleElementPlot(theta1, refPointTrailer, -0.4, -1, 0.4, -1)

% Left rear tyre
SingleElementPlot(theta1, refPointTrailer, -0.4, 1, 0.4, 1)

%% Trailer shape plotting
x1 = refPointTrailer(1); % Extracting coordinates of reference point
y1 = refPointTrailer(2); 
shape1 = polyshape([x1-0.5 x1-0.5 x1+1.5 x1+1.5], [y1+1 y1-1 y1-1 y1+1]); % Normal Trailer shape with geometrical sizes
shape1_rot = rotate(shape1, theta1/(2*pi)*360, refPointTrailer); % Rotating Trailer shape
plot(shape1_rot, 'FaceColor', 'b') % Plot rotating Trailer shape
hold on

end
