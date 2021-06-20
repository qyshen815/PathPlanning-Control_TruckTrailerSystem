function ny = Steering_StabControl(Ref,y_T, Parameters)
%STEERING_OL Summary of this function goes here
%   Detailed explanation goes here
k0 = Parameters.k0;
k1 = Parameters.k1;
k2 = Parameters.k2;
k3 = Parameters.k3;

xRef = Ref.xRef; 
yRef = Ref.yRef; 

dxRef = Ref.dxRef; 
dyRef = Ref.dyRef; 
d2xRef = Ref.d2xRef; 
d2yRef = Ref.d2yRef; 
d3xRef = Ref.d3xRef; 
d3yRef = Ref.d3yRef; 

x1 = y_T(1);    % x coordinate of the trailers rear axle
y1 = y_T(2);    % y coordinate of the trailers rear axle 
ex_t = xRef - x1; 
ey_t = yRef - y1; 
dex_t = n
ny_1 = Ref.d4xref_dsigma4 + k3*; 
ny_2 = Ref.d4yref_dsigma4; 

ny = [ny_1; ny_2]; 

end

