
function coef = PathPlanner(state_x0,state_x1,d0,d1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   Simple Path Planning Function regarding Truck Trailer System
%   state_x0 = start pose
%   state_x1 = end pose
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clarify elements of the state vectors
% state_x0 = [x0_s, y0_s, theta0_s, theta1_s, phi_s];
% state_x1 = [x0_e, y0_e, theta0_e, theta1_e, phi_e];

% Inititalize intital state vector - start pose
x0_s = state_x0(1);
y0_s = state_x0(2);
theta0_s = state_x0(3);
theta1_s = state_x0(4);
phi_s = state_x0(4);

% Inititalize final state vector - end pose
x0_e = state_x1(1);
y0_e = state_x1(2);
theta0_e = state_x1(3);
theta1_e = state_x1(4);
phi_e = state_x1(4);

%%
% Calculate reference values

% yRef and its first, second and third derivatives regarding start pose
yRef_val_s = PathPlannerHelper(x0_s, y0_s, theta0_s, theta1_s, phi_s, d0, d1);

% yRef and its first, second and third derivatives regarding end pose
yRef_val_e = PathPlannerHelper(x0_e, y0_e, theta0_e, theta1_e, phi_e, d0, d1);

%%
% Determine the Coefficents of the polynomial by solving the LGS (Ax = b) 

% Concatenate to vector b (refers to both start and end pose)
b = [yRef_val_s; yRef_val_e]; 

% Matrix (containing 8 conditions)
A = [x0_s^7,    x0_s^6,     x0_s^5,    x0_s^4,    x0_s^3,   x0_s^2,   x0_s,  1;
    7*x0_s^6,   6*x0_s^5,   5*x0_s^4,  4*x0_s^3,  3*x0_s^2, 2*x0_s,   1 ,    0;
    42*x0_s^5,  30*x0_s^4,  20*x0_s^3, 12*x0_s^2, 6*x0_s^1, 2,        0,     0;
    210*x0_s^4, 120*x0_s^3, 60*x0_s^2, 24*x0_s^1, 6,        0,        0,     0;
    x0_e^7,     x0_e^6,     x0_e^5,    x0_e^4,    x0_e^3,   x0_e^2,   x0_e,  1;
    7*x0_e^6,   6*x0_e^5,   5*x0_e^4,  4*x0_e^3,  3*x0_e^2, 2*x0_e,   1 ,    0;
    42*x0_e^5,  30*x0_e^4,  20*x0_e^3, 12*x0_e^2, 6*x0_e^1, 2,        0,     0;
    210*x0_e^4, 120*x0_e^3, 60*x0_e^2, 24*x0_e^1, 6,        0,        0,     0];

% Output (solving the LGS via linsolve) 
coef = linsolve(A, b);

end