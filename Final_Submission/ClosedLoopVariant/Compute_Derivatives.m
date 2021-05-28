% The function returns the first to third derivative of the y coordinate 
% of the path y(x) performed by the by the axle of the last trailer . 


function [yx, yxx, yxxx] = ComputeDerivatives(xs,d0,d1)
% Input State vector xs:
% With elements
%                x,            x coord of center of last trailer rear axle
%                y,            y coord of center of last trailer rear axle
%                \theta0,      orientation of truck
%                \theta1,      orientation of trailer
%                \phi          steering angle

% parameters d0 and d1 - % d0: distance front/rear axle truck

% outputs yx := dy/dx   yxx := d^2y/dx^2   yxxx := d^3y/dy^3
  
    kappa1 = 1./d1 .* tan(xs(3)-xs(4));
    kappa0 = 1./d0 .* tan(xs(5));
  
    yx = tan( xs(4) ); % slope of  trailer

    gamma1 = 1 + yx.^2;
    
    eta1   =  1 + d1.^2.*kappa1.^2 ;
     
   dkappa1_ds1 = (kappa0.*sqrt(eta1) - kappa1 ) .* eta1/d1; 

   dkappa1_dx  =  dkappa1_ds1 .* sqrt(eta1) .* sqrt(gamma1);      %ds1 = sqrt(eta2) ds2 

   yxx = kappa1 .* gamma1.^(3/2);
    

   yxxx = gamma1.^(3/2).*(dkappa1_dx + 3.*yx.*yxx.^2./gamma1^(5/2));
 


    
end




