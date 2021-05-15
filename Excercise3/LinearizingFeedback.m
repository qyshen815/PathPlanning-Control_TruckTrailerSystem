function w = LinearizingFeedback(ny, State, Parameters)
%LINEARIZATIONFEEDBACK Summary of this function goes here
%   Detailed explanation goes here

% assign input parameters
L = Parameters.L; 
theta = State(3); 
xi = State(4); 

% define matrix A
A = [cos(theta),    -((xi.^2)./L).*sin(theta); 
     sin(theta),     ((xi.^2)./L).*cos(theta)]; 

% solve linear equation system
w = linsolve(A,ny); 

end

