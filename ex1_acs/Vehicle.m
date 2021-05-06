
function dState = Vehicle(State,uRef,Parameters)
%VEHICLE Summary of this function goes here
%   Detailed explanation goes here
l_0 = Parameters.l_0; 

x = State(1); 
y = State(2); 
theta = State(3);
v = uRef(2);
phi = uRef(1); 

dState(1,1)=v.*cosd(theta);      % dx
dState(2,1)=v.*sind(theta);      % dy
dState(3,1)=v./l_0.*tan(phi);   % dtheta

end
