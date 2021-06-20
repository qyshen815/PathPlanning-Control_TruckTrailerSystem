function y_T = NonlinearOutputFunction(State, Parameters)
%NONLINEAROUTPUTFUNCTION Summary of this function goes here
%   Detailed explanation goes here

% structure of the nonlinear output function
% y_T = h(x_T) 

% the coordinates of the center of the trailer's axle provide a flat output

% initialize necessary values for formula implementation
d_1 = Parameters.d1;
x_0 = State(1); 
y_0 = State(2); 
theta_1 = State(3);

% define elements of the nonlinear output vector
y_1 = x_0 - d_1*cos(theta_1);
y_2 = y_0 - d_1*sin(theta_1);

% define elements of the nonlinear output vector
y_T = [y_1; 
       y_2]; 

end

