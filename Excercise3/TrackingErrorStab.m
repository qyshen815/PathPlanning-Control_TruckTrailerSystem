
function ny = TrackingErrorStab(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef, Parameters, State)
%STEERINGLAW Summary of this function goes here
%   Detailed explanation goes here

% weighting factors for state feedback (random init) 
% control gains
k0 = Parameters.k0;
k1 = Parameters.k1;

% get x(t) & y(t) by integrating state vector (feedback)
% difference between x(t), y(t) and x(sigma), y(sigma)?
% (due to time-parametrization of vehicle model
x = State(1); 
y = State(2); 
theta = State(3); 
xi = State(4); 

% tracking error dynamics
e_y = yRef - y; 
de_y = dyRef - xi.*sin(theta);  
e_x = xRef - x; 
de_x = dxRef - xi.*cos(theta); 

% state feedback parameters
ny1 = d2xRef + k1.*de_x + k0.*e_x; 
ny2 = d2yRef + k1.*de_y + k0.*e_y; 

% state feedback vector
ny = [ny1; ny2]; 

end
