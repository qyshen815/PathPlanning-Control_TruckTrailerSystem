
function dState = ODEFunc(t, State, Parameters)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


    t = linspace(1,100,1000);
    
    %%
    % calculate reference values of x and y
    [xRef, dxRef, d2xRef, yRef, dyRef, d2yRef] = CalcRefValues(t, Parameters);
    
    %%
    % steering Law calculate input reference values v and phi
    uRef = steeringLaw(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef, Parameters); 
    
    %%
    % simple vehicle model(time parametrized calculated state) 
    dState = Vehicle(State,uRef,Parameters); 
    
end


