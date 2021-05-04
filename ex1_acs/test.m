s = linspace(0,1,1000); 
x = -12*s.^2+10*s; 
y = -0.8429*s.^4 + 1.52*s.^2 + 0.3143*s + 1; 
figure(1)
plot(s,x)
hold on 
plot(s,y)

figure(2) 
plot(x,y)

state_x0 = [0, 0, 0, 0, 0];
state_x1 = [1, -2, 0, 0, 0];
% Function call => get polynomial coefficients
coef = PathPlanner1(state_x0,state_x1)

a = coef(1);
b = coef(2);
c = coef(3);
d = coef(4);
e = coef(5);
f = coef(6);

% Crate and plot the polynomial
x0=0; 
x1=1;
minX = x0; 
maxX = x1;
length = 1000; 
x = linspace(minX,maxX,length);
x_til = linspace(minX-x0,maxX-x0,length);
fx = a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;

x = -12*s.^5 + 30*s.^4 -20*s.^3; 
figure(3)
plot(s,fx)


%%
state_x0 = [0, -2, 0, 0, 0];
state_x1 = [1, 2, 0, 0, 0];
% Function call => get polynomial coefficients
coef = PathPlanner1(state_x0,state_x1)

a = coef(1);
b = coef(2);
c = coef(3);
d = coef(4);
e = coef(5);
f = coef(6);

% Crate and plot the polynomial
x0=0; 
x1=1;
minX = x0; 
maxX = x1;
length = 1000; 
x = linspace(minX,maxX,length);
x_til = linspace(minX-x0,maxX-x0,length);
fy= a.*x_til.^5 + b.*x_til.^4 + c.*x_til.^3 + d.*x_til.^2 + e.*x_til.^1 + f.*x_til.^0;

x = -12*s.^5 + 30*s.^4 -20*s.^3; 
figure(4)
plot(s,fy)

figure(5)
plot(fx,fy)


