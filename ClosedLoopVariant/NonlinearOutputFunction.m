function y_T = NonlinearOutputFunction(State, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function describes the nonlinear output which follows
%   the structure: y_T = h(x_T) 
%   The coordinates of the center of the Trailer's axle provide
%   a flat output.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize necessary values for formula implementation
d_1 = Parameters.d1;
x_0 = State(1); 
y_0 = State(2); 
theta_1 = State(4);

% Define elements of the nonlinear output vector
y_1 = x_0 - d_1*cos(theta_1);
y_2 = y_0 - d_1*sin(theta_1);

% Define nonlinear output vector
y_T = [y_1; 
       y_2]; 

end

