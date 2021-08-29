function w = LinearizingFeedback(ny, State, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function builds the first block of the linearized 
%   system (decoupled integrator chains) and calculates the 
%   input w of the controller state using state feedback.  
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract first four elements of state vector
x_truck = State(1:5);

% Extract last three elements of state vector
x_ctrl = State(6:8); 

% Extract axle distances from parameters
d0 = Parameters.d0;
d1 = Parameters.d1;

% External function to get lie derivatives
LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0, d1);

%%
% Given equation (lecture 6, slide 7)
% ny = A*w + c

% Define matrix A
A = [LD.L_g1_Lf3_h1, LD.L_g2_Lf3_h1;
     LD.L_g1_Lf3_h2, LD.L_g2_Lf3_h2];
 
% Define vector c
c = [LD.Lf4_h1; 
     LD.Lf4_h2]; 
 
% Transform the equation to linsolve structure A*w = b

% Define vector b
b = ny - c; 

% Solve the LGS A*w = b to get w vector
w = linsolve(A, b); 

end



