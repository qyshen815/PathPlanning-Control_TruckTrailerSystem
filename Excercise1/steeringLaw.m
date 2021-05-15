
function uRef = steeringLaw(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef, Parameters)
%STEERINGLAW Summary of this function goes here
%   Detailed explanation goes here

% flat output Y = [x; y] = [y1; y2]
y1 = xRef; 
dy1 = dxRef; 
d2y1 = d2xRef; 

y2 = yRef; 
dy2 = dyRef; 
d2y2 = d2yRef; 

l_0 = Parameters.l_0; 

v = sqrt(dy1.^2 + dy2.^2); 


% define curvature kappa
if v==0
    k=0;
else
    k = (d2y2.*dy1 - d2y1.*dy2) ./ ((dy1.^2 + dy2.^2).^(3/2)); 
end

% steering angle 
phi = atan(l_0.*k); 

% define components of vector u
u1 = phi; 
u2 = v; 

% calculated manipulated variable u
uRef = [u1;u2]; 
end
