
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


% Inititalize intital state vector - start
x0_s = state_x0(1);
y0_s = state_x0(2);
theta0_s = state_x0(3)*360/(2*pi);
theta1_s = state_x0(4)*360/(2*pi);
phi_s = state_x0(4)*360/(2*pi);

% Inititalize final state vector - end
x0_e = state_x1(1);
y0_e = state_x1(2);
theta0_e = state_x1(3)*360/(2*pi);
theta1_e = state_x1(4)*360/(2*pi);
phi_e = state_x1(4)*360/(2*pi);

% start
yRef_val_s = PathPlannerHelper(x0_s, y0_s, theta0_s, theta1_s, phi_s, d0, d1);

% end
yRef_val_e = PathPlannerHelper(x0_e, y0_e, theta0_e, theta1_e, phi_e, d0, d1);

% Coefficents of the polynomial
% LGS (determination of the remaining coefficients)

% concatenate to vector b
b = [yRef_val_s; yRef_val_e]; 

% Matrix
A = [x0_s^7,    x0_s^6,     x0_s^5,    x0_s^4,    x0_s^3,   x0_s^2,   x0_s,  1;
    7*x0_s^6,   6*x0_s^5,   5*x0_s^4,  4*x0_s^3,  3*x0_s^2, 2*x0_s,   1 ,    0;
    42*x0_s^5,  30*x0_s^4,  20*x0_s^3, 12*x0_s^2, 6*x0_s^1, 2,        0,     0;
    210*x0_s^4, 120*x0_s^3, 60*x0_s^2, 24*x0_s^1, 6,        0,        0,     0;
    x0_e^7,     x0_e^6,     x0_e^5,    x0_e^4,    x0_e^3,   x0_e^2,   x0_e,  1;
    7*x0_e^6,   6*x0_e^5,   5*x0_e^4,  4*x0_e^3,  3*x0_e^2, 2*x0_e,   1 ,    0;
    42*x0_e^5,  30*x0_e^4,  20*x0_e^3, 12*x0_e^2, 6*x0_e^1, 2,        0,     0;
    210*x0_e^4, 120*x0_e^3, 60*x0_e^2, 24*x0_e^1, 6,        0,        0,     0];
    
% question pos rear axle trailer/truck

% Output (solving the LGS via linsolve) 
coef = linsolve(A, b);

end