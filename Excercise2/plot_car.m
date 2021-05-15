x = State(:,1);
y = State(:,2);
yaw = State(:,3);  



t = linspace(0, 5*pi, 1000); 
f = sin(t); 


for k = 1:45
   plot(x(k),y(k),'r*') 
   hold on
   plot(x(1:k),y(1:k)) 
   axis([-2 110 -2 110])
   pause(0.1)
   
   if k ~= 45
       clf
   end
end

