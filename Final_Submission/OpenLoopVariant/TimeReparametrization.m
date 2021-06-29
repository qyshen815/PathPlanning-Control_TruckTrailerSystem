function u = TimeReparametrization(w, eta, xRef_dot, x_T, x_C)
%TIMEREPARAMETRIZATION_OL Summary of this function goes here
%   Detailed explanation goes here

% initialize necessary values for formular implementation
xi_1 = x_C(1); 
theta_0 = x_T(3); 
theta_1 = x_T(4);
w_2 = w(2); 

% define input vector elements u1 & u2 for truck-trailer-model
u_1 = xi_1/cos(theta_0 - theta_1) * eta * xRef_dot; 
u_2 = w_2 * eta * xRef_dot;


% define input vector u for truck-trailer-model
u = [u_1; 
     u_2]; 

end

