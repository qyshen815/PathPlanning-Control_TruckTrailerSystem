function dx_C = ControllerState(w, eta, xRef_dot, x_C)
%CONTROLLERSTATE_OL Summary of this function goes here
%   Detailed explanation goes here

% initialize necessary values for formular implementation
xi_2 = x_C(2); 
xi_3 = x_C(3); 
w_1 = w(1); 

% compute controller state
dxi_1 = xi_2*eta*xRef_dot;
dxi_2 = xi_3*eta*xRef_dot;
dxi_3 = w_1*eta*xRef_dot;

% derivate of controller state vector
dx_C = [dxi_1;
        dxi_2;
        dxi_3];

end

