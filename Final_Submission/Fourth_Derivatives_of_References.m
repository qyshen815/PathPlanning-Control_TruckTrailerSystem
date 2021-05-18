function [d4xref_dsigma4, d4yref_dsigma4] = Fourth_Derivatives_of_References(dyx, dyxx, dyxxx, dyxxxx)

%Calculates the fourth derivative of the trajectory subject to the arclength (x'''' and y''''')

% Inputs: 1.,2.,3. and 4. derivative of the reference polynomial subject to
% x, ( dyx, dyxx, dyxxx, dyxxxx )

% Outputs: x'''' and y'''''

eta = sqrt(1 + dyx^2);

d4xref_dsigma4 = -( (2*dyxx*dyxxx + dyxx*dyxxx+ dyx*dyxxxx)/eta^5 - 5/eta^6*dyx*dyxx/eta*(dyxx^2+dyx*dyxxx))/eta ...
                 + ((8*dyx*dyxx^3+8*dyx^2*dyxx*dyxxx)/eta^7 - 7/eta^8*dyx*dyxx/eta*4*dyx^2*dyxx^2 )/eta;
              
d4yref_dsigma4 = 1/eta*(dyxxxx/eta^3 - 3*dyxxx/eta^4*dyx*dyxx/eta - (8*dyxx*dyx*dyxxx + 4*dyxx^3)/eta^5 + 20/eta^6*dyxx^3*dyx^2/eta ...
                         - (2*dyx*dyxx*dyxxx + dyx^2*dyxxxx)/eta^5 + 5*dyx^3*dyxx*dyxxx/eta^7 ...
                         +(12*dyx^2*dyxx^3+ 8*dyx^3*dyxx*dyxxx)/eta^7 - 28*dyx^4*dyxx^3/eta^9);
end

