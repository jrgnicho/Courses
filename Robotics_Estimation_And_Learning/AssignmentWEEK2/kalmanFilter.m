function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 1e+1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    %%
    % State prediction
    % P = A * P_t-1*A_trans + Em
    % R = Eo
    % K = P * C_trans * (R + C * P * C_trans)^-1
    % x_t = A * x_t-1 + K(z_t - C * A * x_t-1)
    % observational matrix C = [1 0 0 0;0 1 0 0;0 0 0 0; 0 0 0 0];
    
    %% Defining matrices 
    n = length(state);
    %dt = t - previous_t; 
    dt = 1;%10 * 0.033;
    
    % transition matrix
    A = eye(length(state));
    A = [1 0 dt 0;0 1 0 dt;0 0 1 0; 0 0 0 1];
    
    % system noise covariance 
    Em = 1e+6 * eye(n);
    
    % measurement noise covariance
    Eo = 1e-1 * eye(n/2);
    
    % Observational matrix
    C = [1 0 0 0;0 1 0 0];
    
    %% Update state covariance P, R matrix and Kalman Gain
    P = param.P;
    P = A * P * (A') + Em;
    R= Eo;
    K = P * (C') * inv(R + C * P * (C'));
    z = [x y 0 0]';
    s = state';
    t1 = C *A * s;
    t2 =  K * (z(1:2,1) - C *A * s);
    t3 = A * s;
    s = t3 + t2;%A * s + K * (z(1:2,1) - C *A * s);    
    P = P - K * C * P;
    
    %% Collecting results
    state = s';
    param.P = P;
    predictx = s(1) + 10* s(3);
    predicty = s(2) + 10* s(4);
    
    %disp('K')
    %disp(K)
    diff = state - z'
    
    return
    
end
