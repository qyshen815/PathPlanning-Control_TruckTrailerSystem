function w = LinearizingFeedback(ny, State, Parameters)
%LINEARIZINGFEEDBACK_OL Summary of this function goes here
%   Detailed explanation goes here

% get first four elements of state vector
x_truck = State(1:5);

% get last three elements of state vector
x_ctrl = State(6:8); 

% get axle distances from parameters
d0 = Parameters.d0;
d1 = Parameters.d1;

% external function to get lie derivatives
LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0, d1);

% equation given on the slide: 
% ny = A*w + c

% define matrix A
A = [LD.L_g1_Lf3_h1, LD.L_g2_Lf3_h1;
     LD.L_g1_Lf3_h2, LD.L_g2_Lf3_h2];

% define vector c
c = [LD.Lf4_h1; 
     LD.Lf4_h2]; 
 
% linsolve structure A*w = b

% define vector b
b = ny - c; 

% solve the lgs A*w = b to get w vector
w = linsolve(A,b); 

end

