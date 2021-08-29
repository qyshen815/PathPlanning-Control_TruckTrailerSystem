function MovingPlot(State, start, final, T, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function file draws a moving plot from the start 
%   pose to the final pose representing the Truck/Trailer
%   trajectory.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Truck coordinates from State vector
x0_mv = State(:,1);
y0_mv = State(:,2);

% Trailer coordinates calculated analogous to the nonlinear output function
x1_mv = State(:,1) - Parameters.d1*cos(State(:,4));
y1_mv = State(:,2) - Parameters.d1*sin(State(:,4));

% Frequency calculated with travel time T
f = 1/T; 

% Last element of iteration
endLoop = length(State); 

% Step width of iteration
stepWidth = round(endLoop/100*14/T); % Scaling step width to determine travel time

%% Iterating loop for moving trajectory
for k = 1 : stepWidth : endLoop
   
   % Determine reference points of Truck/Trailer
   refPointTruck = [x0_mv(k), y0_mv(k)];    % Center of Truck's rear axle
   refPointTrailer = [x1_mv(k), y1_mv(k)];  % Center of Trailer's rear axle
   % Graphical representation of the Truck/Trailer model
   PlotState(refPointTruck, refPointTrailer, State, k)
   hold on
   % Plot an additional marker at the reference points of the Truck/Trailer
   plot(x0_mv(k),y0_mv(k),'r*', 'LineWidth', 1) 
   hold on
   plot(x1_mv(k),y1_mv(k),'b*', 'LineWidth', 1) 
   hold on
   % Plot the resulting trajectory iteratively for Truck and Trailer
   plot(x0_mv(1:k),y0_mv(1:k), 'LineWidth', 1, 'Color', 'r') 
   hold on
   plot(x1_mv(1:k),y1_mv(1:k), 'LineWidth', 1, 'Color', 'b')
   
   if Parameters.direction == 1
      title('Plot 1: Forward Motion - Moving Truck/Trailer')
   else
      title('Plot 1: Backward Motion - Moving Truck/Trailer')
   end
   xlabel('x-coordinate')
   ylabel('y-coordinate')
   %axis([start.x0-5 final.x0+10 start.y0-5 final.y0+10]) % Scaling axis
   grid on
   pause(0.1*f) % Enter frequency
   
   % Iterative plot update
   if k ~= endLoop
       clf    % Clear current figure as long as final value is not reached
   end
end

%% Final plot (moving trajectory has finished) 
refPointTruck = [x0_mv(endLoop), y0_mv(endLoop)];         % Final ref point Truck
refPointTrailer = [x1_mv(endLoop), y1_mv(endLoop)];       % Final ref point Trailer
PlotState(refPointTruck, refPointTrailer, State, endLoop) % Final graphical representation
% endLoop is the last element of the array that is needed to plot the final state
hold on
plot(x0_mv(endLoop),y0_mv(endLoop),'r*', 'LineWidth',1)   % Final marker position Truck
hold on
plot(x1_mv(endLoop),y1_mv(endLoop),'b*', 'LineWidth',1)   % Final marker position Trailer
hold on
plot(x0_mv(1:endLoop),y0_mv(1:endLoop), 'Color', 'r')     % Final trajectory Truck
hold on
plot(x1_mv(1:endLoop),y1_mv(1:endLoop), 'Color', 'b')     % Final trajectory Trailer

if Parameters.direction == 1
      title('Plot 1: Forward Motion - Moving Truck/Trailer')
   else
      title('Plot 1: Backward Motion - Moving Truck/Trailer')
end
xlabel('x-coordinate')
ylabel('y-coordinate')
%axis([start.x0-5 final.x0+10 start.y0-5 final.y0+10])     % Scaling axis 
grid on

end