%% Fake state
state = [3.2, 4.8, 0, 0]';
%% Defining matrices 
    n = length(state);
    dt = t = ;
    
    % transition matrix
    A = [1 0 dt 0;0 1 0 dt;0 0 1 0; 0 0 0 1];
    
    % system noise covariance 
    Em = 1e+1 * eye(n);
    
    % measurement noise covariance
    Eo = 1e-2 * eye(n);
    
    % Observational matrix
    %C = eye(4);
    %C(3:end,3:end) = zeros(2,2);
    C = [1 1];
    
    %% Update state covariance P, R matrix and Kalman Gain
    P = param.P;
    P = A * P * (A') + Em;
    R= Eo;
    K = P * (C') * inv(R + C * P * (C'));
    z = [x y 0 0]';
    x = state;
    x = A * x + K * (z - C *A * x);
    
    %% Collecting results
    state = x';
    param.P = P;
    predictx = state(1);
    predicty = state(2);