function SingleElementPlot_Steering(theta, phi, refPoint, delta_x_s, delta_y_s, delta_x_e, delta_y_e)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file is responsible for the plotting of 
%   single elements of the vehicle (e.g. axles, tyres). This
%   happens by calling the rotation matrix for the corresponding 
%   start and end point. 
%   In addition, it implements the steering of the Truck's front
%   axle. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rotation of main axle to get new reference point (center of Truck's front axle)
[x_s1, y_s1] = CallRotationMatrix(theta, refPoint, 0, 0);
[x_e1, y_e1] = CallRotationMatrix(theta, refPoint, 3, 0); 
newRefPoint = [x_e1, y_e1]; 

% Rotation (steering + orientation angle) according to the new reference point
[x_s2, y_s2] = CallRotationMatrix(phi+theta, newRefPoint, delta_x_s, delta_y_s)  
[x_e2, y_e2] = CallRotationMatrix(phi+theta, newRefPoint, delta_x_e, delta_y_e)

% Plot the single element with implemented steering
plot([x_s2 x_e2], [y_s2 y_e2], 'LineWidth', 2, 'color', 'black')
hold on 
axis equal

end

