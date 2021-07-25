function SingleElementPlot(theta, refPoint, delta_x_s, delta_y_s, delta_x_e, delta_y_e)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file is responsible for the plotting of 
%   single elements of the vehicle (e.g. axles, tyres). This
%   happens by calling the rotation matrix for the corresponding 
%   start and end point. 
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[x_s, y_s] = CallRotationMatrix(theta, refPoint, delta_x_s, delta_y_s); 

[x_e, y_e] = CallRotationMatrix(theta, refPoint, delta_x_e, delta_y_e);

% Plot start and end point of the single element with implemented rotation
plot([x_s x_e], [y_s y_e], 'LineWidth', 2, 'color', 'black')
hold on
axis equal

end

