function posUncertainty = PositionUncertainty()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function calculates a random value for the uncertainty
%   of the start postion within a certain range rounded to one 
%   decimal place. 
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

posUncertainty = round(-5 + (5+5)*rand(1)*10)/10; 

end

