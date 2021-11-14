function angles = kalman_filter(accel, gyro, time)

phi   = zeros(length(time), 1);     % Roll
bias_phi   = zeros(length(time), 1);
theta = zeros(length(time), 1);     % Pitch
bias_theta = zeros(length(time), 1);

P = eye(4);         % error covariance matrix
Q = eye(4) * 0.1;   % covariance matrix of the process noise
R = eye(2) * 10;    % covariance matrix of the measurement noise
state_estimate = [0 0 0 0]';

for t = 2:length(time)
    delta_t = time(t) - time(t-1);
    
    A = [1 -delta_t 0 0; 0 1 0 0; 0 0 1 -delta_t; 0 0 0 1];
    B = [delta_t 0 0 0; 0 0 delta_t 0]';
    C = [1 0 0 0; 0 0 1 0];
    
    p = gyro(t, 1) * pi / 180;
    q = gyro(t, 2) * pi / 180;
    r = gyro(t, 3) * pi / 180;
    
    phi_pre   = phi(t - 1);
    theta__pre = theta(t - 1);
    
    phi_dot   = p + sin(phi_pre) * tan(theta__pre) * q + cos(phi_pre) * tan(theta__pre) * r;
    theta_dot = cos(phi_pre) * q - sin(phi_pre) * r;
          
    % Predict
    state_estimate = A * state_estimate + B * [phi_dot, theta_dot]';
    P = A * P * A' + Q;
    
    % Update
    phi_acc   = atan2( accel(t, 2), sqrt(accel(t, 1) .^ 2 + accel(t, 3) .^ 2));
    theta_acc = atan2(-accel(t, 1), sqrt(accel(t, 2) .^ 2 + accel(t, 3) .^ 2));
    
    measurement = [phi_acc, theta_acc]';
    y_tilde = measurement - C * state_estimate;
    S = C * P * C' + R;
    K = P * C' * (S^-1);
    state_estimate = state_estimate + K * y_tilde;
    P = (eye(4) - K * C) * P;
    
    phi(t)    = state_estimate(1);
    bias_phi(t)   = state_estimate(2);
    theta(t)  = state_estimate(3);
    bias_theta(t) = state_estimate(4);
end

angles = [theta, phi];

end