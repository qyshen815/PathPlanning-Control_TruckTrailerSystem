function LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0,d1)
%
% Calculates the Lie Derivatives of the flat output
%
% Inputs:
% truck state 
%       vector   x,            x coord of center of truck rear axle
%                y,            y coord of center of truck rear axle
%                \theta0,      orientation of truck
%                \theta1,      orientation of trailer
%                \phi          steering angle
% %
% Controller state vector
%                \xi1
%                \xi2
%                \xi3
%
% Structure LD with Lie derivative components  
%          LD.Lf_h1   
%          LD.Lf_h2
%          LD.Lf2_h1
%          LD.Lf2_h2
%          LD.Lf3_h1
%          LD.Lf3_h2
%          LD.Lf4_h1
%          LD.Lf4_h2
%          LD.L_g1_Lf3_h1 
%          LD.L_g2_Lf3_h1
%          LD.L_g1_Lf3_h2 
%          LD.L_g2_Lf3_h2


% Abbreviations
 theta0 = x_truck(3);
 theta1 = x_truck(4);
    phi = x_truck(5);
    
z1 = x_ctrl(1); % xi 1
z2 = x_ctrl(2); % xi 2
z3 = x_ctrl(3); % xi 3
z1sqr = z1*z1;
z1cub = z1sqr*z1;
z1qua = z1cub*z1;

z2sqr = z2*z2;
c0 = cos(theta0); 
c1 = cos(theta1);
s0 = sin(theta0);
s1 = sin(theta1);

t_phi = tan(phi);

t01 = tan(theta0 - theta1);
t01_sqr = t01*t01;
t01_cub = t01_sqr*t01;
%Secants
Sec_phi_2 = 1/cos(phi)^2;
Sec01     = 1/cos(theta0 - theta1);
Sec01_sqr = Sec01*Sec01;
Sec01_cub = Sec01_sqr* Sec01;
Sec01_qua = Sec01_cub* Sec01;
Sec01_qui = Sec01_qua* Sec01;


LD.L_g1_Lf3_h1 = c1;
LD.L_g2_Lf3_h1 = -z1cub * Sec_phi_2*  Sec01_cub * s1/(d0*d1);

LD.L_g1_Lf3_h2 = s1;
LD.L_g2_Lf3_h2 = z1cub * Sec_phi_2*  Sec01_cub * c1/(d0*d1);


 
 
LD.Lf_h1 = z1 * c1;
 
LD.Lf_h2 = z1 * s1;

LD.Lf2_h1 = z2 * c1 - (z1sqr * s1 * t01 )/d1;

LD.Lf2_h2 = z2 * s1 + (z1sqr * c1 * t01)/d1;

LD.Lf3_h1 = z3 * c1... 
         - ( z1cub * s1 * Sec01_cub * t_phi)/(d0 * d1)...
         - ( 3* z1 * z2 * s1 * t01)/d1 ...
         + (  z1cub * s1 *  Sec01_sqr * t01)/d1^2 ...
         - (  z1cub * c1 * t01_sqr)/d1^2;

LD.Lf3_h2 = z3 * s1 ...
         +( z1cub * c1 *Sec01_cub * t_phi)/(d0 * d1) ...
         + (  3 *z1* z2 *c1 * t01)/d1 ...
         - (  z1cub *c1 * Sec01_sqr * t01)/d1^2 ...
         - (  z1cub * s1 * t01_sqr)/d1^2;

LD.Lf4_h1 = -(6*z1sqr * z2* Sec01_cub * s1 *t_phi )/(d0*d1) ...
         +(z1qua * Sec01_qui * s1 * t_phi)/(d0*d1^2)...
         -(3*z2sqr* s1 * t01)/d1...
         -(4*z1*z3*s1 * t01)/d1...
         +(6*z1sqr* z2*Sec01_sqr* s1 * t01)/(d1^2)...
         -(z1qua *Sec01_qua * s1 * t01)/ (d1^3) ...
         -(3*z1qua* c1*Sec01_cub * t_phi * t01)/(d0*d1^2 ) ...
         -(3*z1qua* Sec01_qua * s1 * t_phi^2 *t01)/( d0^2 * d1)...
         -(6*z1sqr* z2* c1 *t01_sqr)/(d1^2) ...
         +(3*z1qua* c1 *Sec01_sqr * t01_sqr)/(d1^3) ...
         +(5*z1qua* Sec01_cub * s1 * t_phi *t01_sqr)/(d0 * d1^2 )...
         +(z1qua* s1 *t01_cub)/(d1^3) ...
         -(2*z1qua* Sec01_sqr * s1 * t01_cub)/(d1^3) ;
     
     
LD.Lf4_h2 = (6*z1sqr* z2 * Sec01_cub * c1 * t_phi)/(d0*d1)...
          -(z1qua* c1*Sec01_qui *t_phi)/(d0*d1^2)...
          +(3*z2sqr *c1*t01)/d1...
          +(4*z1*z3*c1*t01)/d1...
          -(6*z1sqr* z2*c1*Sec01_sqr* t01)/(d1^2) ...
          +(z1qua*  c1*Sec01_qua* t01)/(d1^3) ...
          -(3*z1qua* Sec01_cub* s1*t_phi*t01)/(d0*d1^2)...
          +(3*z1qua* c1*Sec01_qua* t_phi^2* t01)/(d0^2*d1)...
          -(6*z1sqr* z2*s1*t01_sqr)/(d1^2)...
          +(3*z1qua* Sec01_sqr *s1*t01_sqr)/(d1^3)...
          -(5*z1qua* c1*Sec01_cub* t_phi * t01_sqr)/( d0*d1^2)...
          -(z1qua* c1*t01_cub)/(d1^3)...
          +(2*z1qua* c1*Sec01_sqr * t01_cub)/(d1^3) ;
    
end

