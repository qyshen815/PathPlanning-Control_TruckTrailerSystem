function ny = Steering(Ref)
%STEERING_OL Summary of this function goes here
%   Detailed explanation goes here

ny_1 = Ref.d4xref_dsigma4; 
ny_2 = Ref.d4yref_dsigma4; 

ny = [ny_1; ny_2]; 

end

