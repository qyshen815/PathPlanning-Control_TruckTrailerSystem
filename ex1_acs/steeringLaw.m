function uRef = steeringLaw(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef,Parameters)
%STEERINGLAW Summary of this function goes here
%   Detailed explanation goes here

% define components of vector y and its derivatives
y1 = xRef; 
y2 = yRef; 
dy1 = dxRef; 
dy2 = dyRef; 
d2y1 = d2xRef; 
d2y2 = d2yRef;

% helper variable kappa
k = (d2y2.*dy1 - d2y1.*dy2)./((dy1.^2 + dy2.^2).^(3/2)); 
dk = gradient(k); % insecure

dv0 = (dy1.*d2y1 + dy2.*d2y2)./(sqrt(dy1.^2 + dy2.^2)); 
dphi = Parameters.l_0.*dk ./ (1 + Parameters.l_0.^2 .* k.^2);

% define components of vector u
u1 = dphi; 
u2 = dv0; 

% calculated manipulated variable u
uRef = [u1;u2]; 
end

