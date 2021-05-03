function [x,y] = callRotationMatrix(theta, refPoint, dx, dy, phi)
%CALLROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here
X = rotation_matrix(theta, refPoint, dx, dy, 0)

x = X(1) + refPoint(1)
y = X(2) + refPoint(2)
end

