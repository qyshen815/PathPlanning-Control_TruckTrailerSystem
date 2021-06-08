
function coef = PathPlanner(state_x0,state_x1,d0,d1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Simple Path Planning Function
%   state_x0 = inital pose
%   state_x1 = final pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%state_x0 = [x0_s, y0_s, theta0_s, theta1_s, phi_s, v0];
%state_x1 = [x0_e, y0_e, theta0_e, theta1_e, phi_e, v1];


% Inititalize intital state vector 
x0_s = state_x0(1);
y0_s = state_x0(2);
theta0_s = state_x0(3)*360/(2*pi);
theta1_s = state_x0(4)*360/(2*pi);
phi_s = state_x0(4)*360/(2*pi);

% Inititalize final state vector 
x0_e = state_x1(1);
y0_e = state_x1(2);
theta0_e = state_x1(3)*360/(2*pi);
theta1_e = state_x1(4)*360/(2*pi);
phi_e = state_x1(4)*360/(2*pi);

% start
yRef_val_s = PathPlannerHelper(x0_s, y0_s, theta0_s, theta1_s, phi_s);

% end
yRef_val_e = PathPlannerHelper(x0_e, y0_e, theta0_e, theta1_e, phi_e);

% concatinate to vector b
b = [yRef_val_s; yRef_val_e]; 

% Matrix and linsolve...
% question pos rear axle trailer/truck

%{

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
    gamma1^3*1/L*tan(phi1);
    gamma0^3*1/L*tan(phi0);
    tan(theta0);
    y0];

% Output
coef = linsolve(A, b);
%}

end