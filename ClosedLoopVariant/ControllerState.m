function dx_C = ControllerState(w, eta, xRef_dot, x_C)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function determines the states of the decoupling 
%   controller in new parametrization according to arc length
%   (sigma). This includes the calculation of the xi values.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize necessary values for formula implementation
xi_2 = x_C(2); 
xi_3 = x_C(3); 
w_1 = w(1); 

% Compute controller state
dxi_1 = xi_2*eta*xRef_dot;
dxi_2 = xi_3*eta*xRef_dot;
dxi_3 = w_1*eta*xRef_dot;

% Derivate of controller state vector
dx_C = [dxi_1;
        dxi_2;
        dxi_3];
    
end
