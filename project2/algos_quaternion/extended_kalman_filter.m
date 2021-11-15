function quat = extended_kalman_filter(accel, gyro, mag, time)

magDecRad = 0.174532925;
% magDecRad = 0;

% Normalize mag
% mag = normalize(mag, 2, 'norm');

% Initialize quaternion
quat = zeros(length(time), 4);
quat(1, :) = [1, 0, 0, 0];

% Initialize gyro bias
gyro_bias = zeros(length(time), 3);
gyro_bias(1, :) = mean(gyro_bias(1:100, :));

% Initialise covariance matrix
cAtt0 = 0.001;
cBias0 = 0.0001;
P = diag([[1 1 1 1] * cAtt0, [1 1 1] * cBias0] .^ 2);

% Process noise matrix
nProcAtt  = 0.00005;
nProcBias = 0.000001;
Q = diag([[1 1 1 1] * nProcAtt, [1 1 1] * nProcBias] .^ 2);

% Measurement noise matrix
nMeasAcc = 0.05;
nMeasMag = 0.02;
R = diag([[1 1 1] * nMeasAcc, [1 1 1] * nMeasMag] .^ 2);

for t = 2:length(time)-1
    
    % get state
    x_k_minus_1 = [quat(t-1, :), gyro_bias(t-1, :)]';
    
    % predict state
    dq = quaternion(quat(t-1, :)) * quaternion([0, gyro(t-1, :) - gyro_bias(t-1, :)]);
    dq = 1/2 * dq.compact';
    
    delta_t = time(t) - time(t-1);
    x_k_predict = x_k_minus_1 + delta_t * [dq; 0; 0; 0];
    x_k_predict(1:4) = x_k_predict(1:4) / norm(x_k_predict(1:4));
    
    % Compute Jacobian of f, A(x, u)
    A = get_A(quat(t-1, :), gyro(t-1, :), gyro_bias(t-1, :));
    % Update error covariance matrix
    P = P + delta_t * (A * P + P * A' + Q);
    
    % Compute magnetic field unit vector estimate in body coordinates from
    % quaternion estimates
    magUnitEstimate = get_mag_estimation(magDecRad, quat(t-1, :));
    % Compute accelerometer output estimates
    accEstimate = get_acc_estimation(quat(t-1, :), gyro(t-1, :), gyro_bias(t-1, :));
               
    % Output function z(x, u) = [axhat, ayhat, azhat, mxhat, myhat, mzhat]
    z = [accEstimate; magUnitEstimate];  
               
    % Jacobian of z, C(x, u) 
    C = get_C(magDecRad, quat(t-1, :));
    
    % Kalman gain
    K = P * C' / (C * P * C' + R);
    
    % Update error covariance matrix
    P = (eye(length(x_k_predict)) - K * C) * P;
    
    % Update state estimate using measurements (accelerometer and
    % magnetometer)
    z_k = [accel(t, :), mag(t, :)]';
    x = x_k_predict + K * (z_k - z);
    
    quat(t, :) = x(1:4) / norm(x(1:4));
    gyro_bias(t, :) = x(5:7);
    
end

end

function A = get_A(quat, gyro, gyro_bias)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

p = gyro(1);
q = gyro(2);
r = gyro(3);

bp = gyro_bias(1);
bq = gyro_bias(2);
br = gyro_bias(3);

A = [0, -0.5 * (p - bp), -0.5 * (q - bq), -0.5 * (r - br), 0.5 * q1, 0.5 * q2, 0.5 * q3;
     0.5 * (p - bp), 0, 0.5 * (r - br), -0.5 * (q - bq), -0.5 * q0, 0.5 * q3, -0.5 * q2;
     0.5 * (q - bq), -0.5 * (r - br), 0, 0.5 * (p - bp), -0.5 * q3, -0.5 * q0, 0.5 * q1;
     0.5 * (r - br), 0.5 * (q - bq), -0.5 * (p - bp), 0, 0.5 * q2, -0.5 * q1, -0.5 * q0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0];

end

function magUnitEstimate = get_mag_estimation(magDecRad, quat)

smag = sin(magDecRad);
cmag = cos(magDecRad);

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

magUnitEstimate = [-cmag * (2 * q0 * q3 - 2 * q1 * q2) - smag * (2 * q1 * q1 + 2 * q3 * q3 - 1);
                    smag * (2 * q0 * q3 + 2 * q1 * q2) - cmag * (2 * q2 * q2 + 2 * q3 * q3 - 1);
                   -cmag * (2 * q0 * q2 + 2 * q1 * q3) + smag * (2 * q0 * q1 - 2 * q2 * q3)];

end

function accEstimate = get_acc_estimation(quat, gyro, gyro_bias)

Va = 0.3; 
g = 9.81;   % gravity

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

p = gyro(1);
q = gyro(2);
r = gyro(3);

bp = gyro_bias(1);
bq = gyro_bias(2);
br = gyro_bias(3);

accEstimate = [Va * (r - br) - 2 * g * (q2 * q3 + q1 * q0);
               -2 * g * (q1 * q3 - q2 * q0);
               Va * (q - bq) + g * (1 - 2 * (q1 * q1 + q2 * q2))];
end


function C = get_C(magDecRad, quat)

Va = 0.3;
g = 9.81;

smag = sin(magDecRad);
cmag = cos(magDecRad);

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

C = [ -2 * g * q1,  -2 * g * q0,  -2 * g * q3,  -2 * g * q2, 0,   0, -Va;
       2 * g * q2,  -2 * g * q3,   2 * g * q0,  -2 * g * q1, 0,   0,   0;
                0,  -4 * g * q1,  -4 * g * q2,            0, 0, -Va,   0;
     -2 * q3 * cmag, 2 * q2 * cmag - 4 * q1 * smag, 2 * q1 * cmag, -2 * q0 * cmag - 4 * q3 * smag, 0, 0, 0;
      2 * q3 * smag, 2 * q2 * smag, 2 * q1 * smag - 4 * q2 * cmag, 2 * q0 * smag - 4 * q3 * cmag, 0, 0, 0;
     -2 * q2 * cmag + 2 * q1 * smag, -2 * q3 * cmag + 2 * q0 * smag, -2 * q0 * cmag - 2 * q3 * smag, -2 * q1 * cmag - 2 * q2 * smag, 0, 0, 0];

end