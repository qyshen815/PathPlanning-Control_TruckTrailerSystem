function dx_T = TruckTrailerModel(x_T, u, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function describes the model of the Truck/Trailer 
%   system including x-/y-coordinates, orientation and steering
%   angles and is the plant of the control system. 
%   Its structure can be seen in the following equation: 
%       dx_T = f(x_T, u).
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize necessary values for formula implementation
d_0 = Parameters.d0;
d_1 = Parameters.d1;
theta_0 = x_T(3); 
theta_1 = x_T(4);
phi = x_T(5); 
u_1 = u(1); % u_1 = v_0
u_2 = u(2);

% Define elements of the Truck/Trailer vector (vehicle model)
dx_0 = u_1*cos(theta_0); 
dy_0 = u_1*sin(theta_0); 
dtheta_0 = u_1/d_0 * tan(phi); 
dtheta_1 = u_1/d_1 * sin(theta_0 - theta_1);
dphi = u_2; 

% Define Truck/Trailer vector (vehicle model)
dx_T = [dx_0; 
        dy_0; 
        dtheta_0; 
        dtheta_1; 
        dphi]; 

end


