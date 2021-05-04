

clear
close
clc 

figure(1)
%plot([1 3], [2 4], 'LineWidth', 2.0)
%axis([0 10 0 10])

x0 = 33; 
y0 = 10; 
shape = polyshape([x0-2 x0-2 x0+18 x0+18], [y0+5 y0-5 y0-5 y0+5]);
r = rotate(shape, 0, [x0,y0])
plot(r)
axis([0 100 0 100])
hold on
grid on

plot([33 46.9], [10 18], 'LineWidth', 2.0, 'color', 'black')
plot([30.5 35.5], [14.4 5.6], 'LineWidth', 2.0, 'color', 'black')

%
%plot(44.4,22.4, '*')

theta = 0;
refPoint = [33.0,10.0];

dx = 16; 
dy = -5; 
X = rotatation_matrix(theta,refPoint,dx,dy)
x_0 = X(1) + refPoint(1)
y_0 = X(2) + refPoint(2)

dx = 16; 
dy = 5; 
X = rotatation_matrix(theta,refPoint,dx,dy)
x_1 = X(1) + refPoint(1)
y_1 = X(2) + refPoint(2)

plot([x_0 x_1], [y_0 y_1], 'LineWidth', 2.0, 'color', 'black')