function ny = Steering_StabControl(Ref, State, y_T, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file describes the steering law and 
%   determines the input values (vector ny) for feedback 
%   linearization. Additionally, it implements a stabilization 
%   control by using the feedback of state and flat output. 
%   Therefore, the function takes the tracking error dynamics
%   into account. 
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read weighting factors from struct 
k0 = Parameters.k0;
k1 = Parameters.k1;
k2 = Parameters.k2;
k3 = Parameters.k3;

% Read distances between axles from struct
d0 = Parameters.d0; 
d1 = Parameters.d1; 

% Split state vector in controller and vehicle state 
x_truck = State(1:5); 
x_ctrl = State(6:8); 

% Set references from CalcRefValues.m
xRef = Ref.xRef; 
dxRef = Ref.dxref_dsigma;
d2xRef = Ref.d2xref_dsigma2;
d3xRef = Ref.d3xref_dsigma3;

yRef = Ref.yRef; 
dyRef = Ref.dyref_dsigma;  
d2yRef = Ref.d2yref_dsigma2;  
d3yRef = Ref.d3yref_dsigma3; 

%%
% Extract actual values from Truck_1T_LieDeriv.m
LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0,d1);

dx_t = LD.Lf_h1; 
d2x_t = LD.Lf2_h1; 
d3x_t = LD.Lf3_h1; 

dy_t = LD.Lf_h2; 
d2y_t = LD.Lf2_h2; 
d3y_t = LD.Lf3_h2; 

%%
% y(T) equals current Trailer position
x1 = y_T(1);    % x coordinate of the Trailer's rear axle
y1 = y_T(2);    % y coordinate of the Trailer's rear axle 

% Tracking error dynamics
% Difference between reference and actual values
ex_t = xRef - x1; 
dex_t = dxRef - dx_t;
d2ex_t = d2xRef - d2x_t;
d3ex_t = d3xRef - d3x_t;

ey_t = yRef - y1; 
dey_t = dyRef - dy_t;
d2ey_t = d2yRef - d2y_t;
d3ey_t = d3yRef - d3y_t;

% Equations of the controller for feedback linearization (input) 
ny_1 = Ref.d4xref_dsigma4 + k3*d3ex_t + k2*d2ex_t + k1*dex_t + k0*ex_t; 
ny_2 = Ref.d4yref_dsigma4 + k3*d3ey_t + k2*d2ey_t + k1*dey_t + k0*ey_t;

ny = [ny_1; ny_2]; 

end

