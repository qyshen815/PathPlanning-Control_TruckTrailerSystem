
function dState = ODEFunc(t, State, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Course/Lecturer: Automotive Control Systems / Wirtensohn
%   Authors:         N. Kugler, M. Reichelt
%
%   This function is the main component and represents
%   the single elements of the block diagram (open loop variant),
%   thereby putting all the functionality together.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%
    % Calculate reference values of x and y
    [Ref, eta, xRef_dot] = CalcRefValues(t, Parameters);
    
    %%
    % steering Law calculate input reference values v and phi
    y_T = [0;0];
    ny = Steering_StabControl(Ref,State,y_T, Parameters); 
    
    %%
    % Linear Feedback Controller using state feedback
    % first block of the linearized system (decoupled integrator chains)
    w = LinearizingFeedback(ny, State, Parameters); 
    
    %%
    % Controller State/Decoupling Controller in new parametrization (sigma)
    x_C = State(6:8);  % x_C equals xi vector [xi_1, xi_2, xi_3]
    dx_C = ControllerState(w, eta, xRef_dot, x_C); 
    
    %% 
    % Reparametrization of the subject (change in time replacing the arc
    % length)
    x_T = State(1:5); % x_T equals Truck vector [x0, y0, theta0, theta1, phi]
    u = TimeReparametrization(w, eta, xRef_dot, x_T, x_C); 
    
    %%
    % Simple vehicle model (time parametrized calculated state) 
    dx_T = TruckTrailerModel(x_T, u, Parameters);
    
    %%
    % Concatenate Truck and xi vector to State = [x_T; x_C]
    [State, dState] = StateConcatenate(x_T, x_C, dx_T, dx_C);
    
    %% 
    % Nonlinear output function y_T = h(x_T)
    y_T = NonlinearOutputFunction(State, Parameters);
    
end
