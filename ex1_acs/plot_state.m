function plot_state(refPoint, theta, phi)
%PLOT_STATE Summary of this function goes here
%   Detailed explanation goes here
dx = 0; 
dy = 0; 
[x_0, y_0] = callRotationMatrix(theta, refPoint, dx, dy, 0); 
dx = 16; 
dy = 0; 
[x_1, y_1] = callRotationMatrix(theta, refPoint, dx, dy, 0);
plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
dx = 0;
dy = -5; 
[x_0, y_0] = callRotationMatrix(theta, refPoint, dx, dy, 0);
dx = 0;  
dy = 5; 
[x_1, y_1] = callRotationMatrix(theta, refPoint, dx, dy, 0);
plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
hold on

dx = 16; 
dy = -5; 
%refPoint = [x0,y0];
X = rotation_matrix(theta, refPoint, dx, dy,0)
[x_0, y_0] = callRotationMatrix(theta, refPoint, dx, dy, 0);
dx = 16; 
dy = 5; 
[x_1, y_1] = callRotationMatrix(theta, refPoint, dx, dy, 0);
plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
end