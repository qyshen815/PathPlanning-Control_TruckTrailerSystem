function u = TimeReparametrization(w, eta, xRef_dot, x_T, x_C)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file implements the time dependency 
%   by re-parametrizing the subject change in time replacing
%   the arc length sigma. 
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize necessary values for formula implementation
xi_1 = x_C(1); 
theta_0 = x_T(3); 
theta_1 = x_T(4);
w_2 = w(2); 

% Define input vector elements u1 & u2 for Truck/Trailer model
u_1 = xi_1/cos(theta_0 - theta_1) * eta * xRef_dot; 
u_2 = w_2 * eta * xRef_dot;

% Define input vector u for Truck/Trailer model
u = [u_1; 
     u_2]; 

end

