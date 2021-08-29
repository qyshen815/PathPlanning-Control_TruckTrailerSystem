function [State, dState] = StateConcatenate(x_T, x_C, dx_T, dx_C)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   Concatenate the vehicle and controller vector to
%   one state vector: 
%       State = [x_T; x_C].
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

State = [x_T; x_C]; 
dState = [dx_T; dx_C]; 

end

