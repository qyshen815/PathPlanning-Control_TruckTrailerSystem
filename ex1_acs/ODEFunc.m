function dState = ODEFunc(t, State, Parameters)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    [xRef, dxRef, d2xRef, yRef, dyRef, d2yRef] = CalcRefValues(t, Parameters);
    
    uRef = steeringLaw(xRef,dxRef,d2xRef,yRef,dyRef,d2yRef,Parameters); 
    dState = Vehicle(State,uRef,Parameters); 
    
end

