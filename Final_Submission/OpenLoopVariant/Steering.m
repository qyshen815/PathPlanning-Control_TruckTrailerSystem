function ny = Steering(Ref)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file describes the steering law and 
%   determines the input values (vector ny) for feedback 
%   linearization.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open loop variant: no stabilization control,
%                    simply assigning ny to the 4th derivative
%                    of the ref values
ny_1 = Ref.d4xref_dsigma4; 
ny_2 = Ref.d4yref_dsigma4; 

ny = [ny_1;
      ny_2]; 
  
end

