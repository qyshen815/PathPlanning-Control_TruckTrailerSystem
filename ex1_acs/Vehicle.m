
function dState = Vehicle(State,uRef,Parameters)
%VEHICLE Summary of this function goes here
%   Detailed explanation goes here
dx = State(1); 
dy = State(2); 
dtheta = State(3); 
v0 = uRef(2);
phi = uRef(1); 

dState(1,1)=v0*cos(dtheta);
dState(2,1)=v0*sin(dtheta);
dState(3,1)=v0/Parameters.l_0*tan(phi);

end