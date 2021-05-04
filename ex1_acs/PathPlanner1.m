function coef = PathPlanner1(state_x0,state_x1,l_0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Simple Path Planning Function
%   state_x0 = inital pose
%   state_x1 = final pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

l_0=2; % Length between axles in m

% Inititalize intital state vector 
x0 = state_x0(1);
y0 = state_x0(2);
theta0 = state_x0(3)*360/(2*pi);
phi0 = state_x0(4)*360/(2*pi);
v0 = state_x0(5);

% Inititalize final state vector 
x1 = state_x1(1);
y1 = state_x1(2);
theta1 = state_x1(3)*360/(2*pi);
phi1 = state_x1(4)*360/(2*pi);
v1 = state_x1(5);

% Helper variables
gamma0 = sqrt(tan(theta0)^2+1); % gamma at x0
gamma1 = sqrt(tan(theta1)^2+1); % gamma at x1
x1_til = x1 - x0;   % diff between x-coordinates

% Coefficents of the polynomial
% LGS (determination of the remaining coefficients)

% Matrix
A = [x1_til^5, x1_til^4, x1_til^3, x1_til^2, x1_til, 1;
    5*x1_til^4, 4*x1_til^3, 3*x1_til^2, 2*x1_til, 1 , 0;
    20*x1_til^3, 12*x1_til^2, 6*x1_til^5, 2, 0, 0;
    0, 0, 0, 2, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];

% Vector
b = [y1; 
    tan(theta1); 
    gamma1^3*1/l_0*tan(phi1);
    gamma0^3*1/l_0*tan(phi0);
    tan(theta0);
    y0];

% Output
coef = linsolve(A, b);

end

