function [xRef, dxRef, d2xRef, yRef, dyRef, d2yRef] = CalcRefValues(t,Parameters)
%CALCREFVALUES Summary of this function goes here
%   Detailed explanation goes here

a = Parameters.coef(1);
b = Parameters.coef(2);
c = Parameters.coef(3);
d = Parameters.coef(4);
e = Parameters.coef(5);
f = Parameters.coef(6);

% Time parametrization
tau = t./Parameters.T; 
x1_til = Parameters.x1-Parameters.x0;

% defining a scaling parameter and its derivatives
s_tau = 3.*tau.^2 - 2.*tau.^3; 
ds_tau = gradient(s_tau)./gradient(tau); 
d2s_tau = gradient(ds_tau)./gradient(tau); 

% possible time parametrization of x and its derivatives
xRef = Parameters.x0 + x1_til.*s_tau; 
dxRef = 1./Parameters.T.*x1_til.*ds_tau;
d2xRef = 1./Parameters.T.^2.*x1_til.*d2s_tau;

% create the polynomial
minX = x0; 
maxX = x1;
length = 1000; 
%x = linspace(minX,maxX,length);
x_til = linspace(minX-x0,maxX-x0,length);


% possible time-parametrization of y and its derivatives
yRef = a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;

% first derivative without time parameter
dy_dx = gradient(yRef)./gradient(x_til);
dyRef = dy_dx.*dxRef;

% second derivative without time parameter
d2y_dx2 = gradient(dy_dx)./gradient(x_til);
d2yRef = d2y_dx2.*dxRef.^2 + dy_dx.*d2xRef; 

end

