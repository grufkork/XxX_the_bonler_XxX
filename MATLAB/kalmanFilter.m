function [x, P] = kalmanFilter(y, H, R, x_prev, P_prev, A, Q, B, u_prev)

    % Input:
    %   y           [m x 1] Current measurement
    %   u           [n x r] previous iteration input
    %   x0          [n x 1] Initial prior mean
    %   P0          [n x n] Initial prior covariance
    %   A           [n x n] State transition matrix
    %   Q           [n x n] Process noise covariance
    %   H           [m x n] Measurement model matrix
    %   R           [m x m] Measurement noise covariance

    % Output:
    %   x           [n x 1] Estimated state vector
    
    % Prediction
    x_pred = A*x_prev + B*u_prev;   % Predicted state
    P = A*P_prev*A.' + Q;           % Filter error covariance
    
    % Measurement update
    S = H*P*H.' + R;            % Innovation covariance
    K = P*H.' / S;              % Kalman gain
    v = y - H*x_pred;           % Innovation/corrected measurement
    
    % Update filter estimate
    x = x_pred + K*v;           % Predicted state corrected by kalman gain
    P = P - K*S*K.';            % Cov is reduced by measurement

end