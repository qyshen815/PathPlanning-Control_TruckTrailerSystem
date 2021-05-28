
function [Ref, eta, xRef_dot] = CalcRefValues(t, Parameters)
%CALCREFVALUES Summary of this function goes here

%   Open Loop method

a = Parameters.coef(1);
b = Parameters.coef(2);
c = Parameters.coef(3);
d = Parameters.coef(4);
e = Parameters.coef(5);
f = Parameters.coef(6);
x0 = Parameters.x0;
x1 = Parameters.x1; 
T = Parameters.T; 



% Time parametrization
tau = t./T; 

% defining a scaling parameter and its derivatives
s_tau = 3.*tau.^2 - 2.*tau.^3; 
ds_tau = 6.*tau - 6*tau.^2;  
%d2s_tau = 6 - 12.*tau; 

% possible time parametrization of x and its derivatives
xRef = x0 + (x1-x0).*s_tau; 
xRef_dot = 1./T.*(x1-x0).*ds_tau; % x dot
%d2xRef = 1./T.^2.*(x1-x0).*d2s_tau;

% helper variable to derive the polynomial
x_til = xRef - x0;
%dx_til = dxRef_t - x0; 

% polynomial with time parametrization 
yRef = a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;

% first derivative without time parameter
dyx = 5.*a.*x_til.^4 + 4.*b.*x_til.^3 + 3.*c.*x_til.^2 + 2.*d.*x_til + e;
% second derivative without time parameter
dyxx = 20.*a.*x_til.^3 + 12.*b.*x_til.^2 + 6.*c.*x_til + 2.*d; 
% third derivative without time parameter
dyxxx = 60.*a.*x_til.^2 + 24.*b.*x_til + 6.*c;
% fourth derivative without time parameter
dyxxxx = 120.*a.*x_til + 24.*b;

% determine eta
eta = sqrt(1 + dyx^2); 

% determine the derivatives of the ref values
dxref_dsigma = 1/eta; 
d2xref_dsigma2 = -(dyx*dyxx)/(eta^4);
d3xref_dsigma3 = -((dyxx^2) + dyx*dyxxx) / (eta^5) ...
                 + (4*(dyx^2)*(dyxx^2)) / (eta^7); 

dyref_dsigma = dyx/eta; 
d2yref_dsigma2 = dyxx/(eta^2) - ((dyx^2)*dyxx)/(eta^4); 
d3yref_dsigma3 = dyxxx/(eta^3) + (4*(dyxx^2)*dyx)/(eta^5) ...
                - ((dyx^2)*dyxxx)/(eta^5) + (4*(dyx^3)*(dyxx^2))/(eta^7);
            
[d4xref_dsigma4, d4yref_dsigma4] = Fourth_Derivatives_of_References(dyx, dyxx, dyxxx, dyxxxx, eta);

% define struct for reference values
Ref.xRef = xRef; 
Ref.dxref_dsigma = dxref_dsigma; 
Ref.d2xref_dsigma2 = d2xref_dsigma2;
Ref.d3xref_dsigma3 = d3xref_dsigma3; 
Ref.d4xref_dsigma4 = d4xref_dsigma4; 

Ref.yRef = yRef; 
Ref.dyref_dsigma = dyref_dsigma; 
Ref.d2yref_dsigma2 = d2yref_dsigma2; 
Ref.d3yref_dsigma3 = d3yref_dsigma3;
Ref.d4yref_dsigma4 = d4yref_dsigma4;

end
