
function [dState, y_T] = ODEFunc(t, State, Parameters)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    
    %%
    % calculate reference values of x and y
    [Ref, eta, xRef_dot] = CalcRefValues(t, Parameters);
    
    %%
    % steering Law calculate input reference values v and phi
    ny = Steering(Ref); 
    
    %%
    % 
    w = LinearizingFeedback(ny, State, Parameters); 
    
    %%
    % Controller State
    % x_C equals xi vector [xi_1, xi_2, xi_3]
    dx_C = ControllerState(w, eta, xRef_dot, x_C); 
    
    %% 
    %
    u = TimeReparametrization(w, eta, xRef_dot, x_T, x_C); 
    
    %%
    % simple vehicle model(time parametrized calculated state) 
    dx_T = TruckTrailerModel(x_T, u, Parameters);
    
    %%
    % State = [x_T; x_C]
    [State, dState] = StateConcatenate(x_T, x_C, dx_T, dx_C);
    
    %% 
    % Nonlinear output function
    y_T = NonlinearOutputFunction(State, Parameters);
    
end

