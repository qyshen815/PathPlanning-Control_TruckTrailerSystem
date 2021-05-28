function [State, dState] = StateConcatenate(x_T, x_C, dx_T, dx_C)
%STATECONCATENATE_OL Summary of this function goes here
%   Detailed explanation goes here

% concatenate the vehicle and controller vector to one state vector
State = [x_T; x_C]; 
dState = [dx_T; dx_C]; 

end

