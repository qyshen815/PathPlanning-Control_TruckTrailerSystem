
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Kurs/Dozent: Automotive Control Systems / Wirtensohn
%   Autoren:     N. Kugler, M. Reichelt
%
%   Simple Path Planning 
%   main file (call function PathPlanner1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear former data
clear
close
clc

% test scenarios
% 1 1 0 0 | 100 -30 0 0
% 1 1 0 0 | 100 100 0 0
% 1 1 45 0 | 100 100 0 0
% 1 1 45 0 | 100 100 90 0

% distance between front and rear axis
l_0 = 2; 

% Define and initialize variables
x0 = 1; 
y0 = 1; 
theta0_d = 0;
theta0 = theta0_d*2*pi/360;
phi0_d = 0; 
phi0 = phi0_d*2*pi/360; 
v0 = 1;

x1 = 100; 
y1 = -30;  
theta1_d = 0;
theta1 = theta1_d*2*pi/360;
phi1_d = 0; 
phi1 = phi1_d*2*pi/360; 
v1 = 1;

% Build the vectors for initial and final pose of the vehicle
state_x0 = [x0, y0, theta0, phi0, v0];
state_x1 = [x1, y1, theta1, phi1, v1];

% Function call => get polynomial coefficients
coef = PathPlanner1(state_x0,state_x1,l_0)

a = coef(1);
b = coef(2);
c = coef(3);
d = coef(4);
e = coef(5);
f = coef(6);

% create the polynomial
minX = x0; 
maxX = x1;
length = 1000; 
x = linspace(minX,maxX,length);
x_til = linspace(minX-x0,maxX-x0,length);

fx = a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;
dfx = gradient(fx)./gradient(x); 
d2fx = gradient(dfx)./gradient(x); 



poly = [a b c d e f];

% Helper variables
x1_u = ((x1-x0)/3*1)-2;
x1_o = ((x1-x0)/3*1)+18;
f1_u = polyval(poly, x1_u);
f1_o = polyval(poly, x1_o);

x2_u = ((x1-x0)/3*2)-2;
x2_o = ((x1-x0)/3*2)+18;
f2_u = polyval(poly, x2_u);
f2_o = polyval(poly, x2_o);

x3_u = ((x1-x0)/3*3)-2;
x3_o = ((x1-x0)/3*3)+18;
f3_u = polyval(poly, x3_u);
f3_o = polyval(poly, x3_o);

% Define angle
% test code


k1 = round((1/3)*100);
val = interp1(x,dfx,k1);
%theta_1 = atand(val);
theta_1 = atan(val); 

val = interp1(x,dfx,k1+16);
%theta_11 = atand(val); 
theta_11 = atan(val);

k2 = round((2/3)*100); 
val = interp1(x,dfx,k2);
%theta_2 = (360*atan(val))/(2*pi);
theta_2 = atan(val); 

val = interp1(x,dfx,k2+16);
%theta_22 = (360*atan(val))/(2*pi);
theta_22 = atan(val); 

% steering angle
gamma = sqrt(tan(theta1)^2+1);
phi_f1 = atan(d2fx(333)*2*1/gamma^3);
phi_f2 = atan(d2fx(666)*2*1/gamma^3);


% put into loop
% Define rectangles (=shape of vechicles)
shape0 = polyshape([x0-2 x0-2 x0+18 x0+18], [y0+5 y0-5 y0-5 y0+5]);
r0 = rotate(shape0, theta0*360/(2*pi), [x0 y0]);
shape1 = polyshape([x1_u-2 x1_u-2 x1_o-2 x1_o-2], [f1_u+5 f1_u-5 f1_u-5 f1_u+5]);
r1 = rotate(shape1, theta_1*360/(2*pi), [x1_u, f1_u]);
shape2 = polyshape([x2_u-2 x2_u-2 x2_o-2 x2_o-2], [f2_u+5 f2_u-5 f2_u-5 f2_u+5]);
r2 = rotate(shape2, theta_2*360/(2*pi), [x2_u f2_u]);
shape3 = polyshape([x3_u+1 x3_u+1 x3_o+1 x3_o+1], [f3_u+5 f3_u-5 f3_u-5 f3_u+5]);
r3 = rotate(shape3, theta1_d, [x1 y1]);

% plot the polynoms
%{
figure(1) 
plot(x,fx); 
hold on; 
grid on; 
plot(x,dfx)
plot(x,d2fx)
hold off; 
%}

% Plot the path
figure(2)
plot(x, fx, 'LineWidth', 2.0);
xlabel('x in meters');
ylabel('y in meters');
title('Simple Path Planning');
grid on;
hold on;
%plot(x,dfx)

%rectangle('Position', [x0-2 y0-5 20 10]);
plot([r0 r1 r2 r3]);

% state r0
refPoint = [x0,y0];
plot_state(refPoint, theta0, 0)  %last for phi

% state r1
refPoint = [x1_u, f1_u];
plot_state(refPoint, theta_1, 0)
% leave comment because of steering of front axis
%{
dx = 0; 
dy = 0; 
refPoint = [x1_u, f1_u];
X = rotation_matrix(theta_1, refPoint, dx, dy,0)
x_0 = X(1) + refPoint(1)
y_0 = X(2) + refPoint(2)
dx = 16; 
dy = 0; 
X = rotation_matrix(theta_1,refPoint,dx,dy,0)
x_1 = X(1) + refPoint(1)
y_1 = X(2) + refPoint(2)
plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
w1 = x_1
w2 = y_1


dx = 0; 
dy = -5; 
refPoint = [x1_u, f1_u];
X = rotation_matrix(theta_1, refPoint, dx, dy,0)
x_0 = X(1) + refPoint(1)
y_0 = X(2) + refPoint(2)
dx = 0; 
dy = 5; 
X = rotation_matrix(theta_1,refPoint,dx,dy,0)
x_1 = X(1) + refPoint(1)
y_1 = X(2) + refPoint(2)
plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
hold on


dx = 16; 
dy = -5; 
refPoint = [x_1, y_1];
X = rotation_matrix(theta_1, refPoint, dx, dy,0)
x_0 = X(1) + refPoint(1)
y_0 = X(2) + refPoint(2)
dx = 16; 
dy = 5; 
X = rotation_matrix(theta_1,refPoint,dx,dy,0)
x_1 = X(1) + refPoint(1)
y_1 = X(2) + refPoint(2)



dx = 0; 
dy = -5; 
refPoint = [w1,w2]%[45.3, 11.8];
X = rotation_matrix(theta_1+phi_f1, refPoint, dx, dy,0)
x_0 = X(1) + refPoint(1)
y_0 = X(2) + refPoint(2)
dx = 0; 
dy = 5; 
X = rotation_matrix(theta_1+phi_f1,refPoint,dx,dy,0)
x_1 = X(1) + refPoint(1)
y_1 = X(2) + refPoint(2)


plot([x_0 x_1], [y_0 y_1], 'LineWidth', 4.0, 'color', 'black')
%}

% state r2
refPoint = [x2_u, f2_u];
plot_state(refPoint, theta_2, 0)
%plot(....)


% state r3 
refPoint = [x1,y1];
plot_state(refPoint, theta1, 0)

axis equal;
