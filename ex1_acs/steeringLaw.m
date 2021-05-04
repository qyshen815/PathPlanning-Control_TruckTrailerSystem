
function uRef = steeringLaw(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef, Parameters)
%STEERINGLAW Summary of this function goes here
%   Detailed explanation goes here

l_0 = Parameters.l_0; 

dv0 = sqrt(dxRef.^2 + dyRef.^2); 
dphi = atan(l_0.*(dxRef.*d2xRef - d2xRef.*dyRef)./((dxRef.^2 + dyRef.^2).^(3/2))); 

% define components of vector u
u1 = dphi; 
u2 = dv0; 

% calculated manipulated variable u
uRef = [u1,u2]; 
end