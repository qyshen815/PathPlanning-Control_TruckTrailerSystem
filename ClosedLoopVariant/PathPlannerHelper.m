function yRef_val = PathPlannerHelper(x0, y0, theta0, theta1, phi, d0, d1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This helper function calculates the y-reference values
%   based on the pose and the geometry of the Truck/Trailer
%   System (coordinates, orientation and steering angles, 
%   axle distances).
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Helper variables - first state equation
eta = 1/cos(theta1); 
x0_x = eta/cos(theta0 - theta1)*cos(theta0); 
y0_x = eta/cos(theta0 - theta1)*sin(theta0);
theta0_x = 1/d0 * eta/cos(theta0 - theta1) * tan(phi);
theta1_x = 1/d1 * eta * tan(theta0 - theta1);

% Express yRef and its derivatives only dependent on the state elements
yRef = y0 - d1*sin(theta1); % With regard to rear axle of Trailer
yRef_x = tan(theta1); 
yRef_xx = eta^3 * (1/d1) * tan(theta0 - theta1); 

eta_x = yRef_x*yRef_xx / eta; % Additional helper variable

yRef_xxx = 3*eta^2*eta_x*(1/d1)*tan(theta0 - theta1) ...
         + eta^3*(1/d1)*(1/((cos(theta0 - theta1))^2))*(theta0_x - theta1_x);    
     
yRef_val = [yRef; 
            yRef_x; 
            yRef_xx;
            yRef_xxx]; 
end



