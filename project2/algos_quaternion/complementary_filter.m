function quat = complementary_filter(accel, gyro, mag, time)
% Reference: Inertial and Magnetic Posture Tracking for Inserting Humans 
% Into Networked Virtual Environments
% Link: https://calhoun.nps.edu/handle/10945/41586

% Normalize mag
mag = normalize(mag, 2, 'norm');

% Initialize gravity
m = [0, 0, 1];
% Initialize mag
n = [1, 0, 0];
% Initialize quaternion
quat = zeros(length(time), 4);
quat(1, :) = [1, 0, 0, 0];

% Define fileter gain
k = 0.001;

for t = 2:length(time)-1
    % measurement vector
    y0 = [accel(t-1, :), mag(t-1, :)]';
    
    q_pre = quat(t-1, :);
    % transfer m and n from the earth fixed frame to the body frame
    h_pre = quatrotate(q_pre, m);
    b_pre = quatrotate(q_pre, n);
    % computed measurement vector
    y_pre = [h_pre, b_pre]';
    
    % calculate the modeling error 
    % (the difference between the actual and the computed measurement)
    error = y0 - y_pre;
    
    % calculate correction
    % obtain matrix X (6*4 matrix, dy_pre dq, equation 12)
    X = get_X(q_pre, m, n);
    
    % equation 11
    delta_q = (X' * X) \ (X' * error);      
    delta_t = time(t) - time(t-1);
    % equation 1
    dq = quaternion(q_pre) * quaternion([0, gyro(t-1, :)]);
    dq = 1/2 * dq.compact;
    
    % equation 15
    q_new = q_pre + k * delta_t * delta_q' + dq * delta_t;
    % normalize the quaternion
    quat(t, :) = q_new / norm(q_new);
end

end


function X = get_X(q_pre, m, n)

q0 = q_pre(1);
qx = q_pre(2);
qy = q_pre(3);
qz = q_pre(4);

m1 = m(1);
m2 = m(2);
m3 = m(3);

n1 = n(1);
n2 = n(2);
n3 = n(3);

X = zeros(6, 4);

X(1, 1) = 4*m1*q0 - 2*m3*qy + 2*m2*qz;
X(1, 2) = 4*m1*qx + 2*m2*qy + 2*m3*qz;
X(1, 3) =           2*m2*qx - 2*m3*q0;
X(1, 4) =           2*m2*q0 + 2*m3*qx;

X(2, 1) = 4*m2*q0 + 2*m3*qx - 2*m1*qz;
X(2, 2) =           2*m3*q0 + 2*m1*qy;
X(2, 3) = 2*m1*qx + 4*m2*qy + 2*m3*qz;
X(2, 4) =           2*m3*qy - 2*m1*q0;

X(3, 1) = 4*m3*q0 - 2*m2*qx + 2*m1*qy;
X(3, 2) =           2*m1*qz - 2*m2*q0;
X(3, 3) =           2*m1*q0 + 2*m2*qz;
X(3, 4) = 2*m1*qx + 2*m2*qy + 4*m3*qz;

X(4, 1) = 4*n1*q0 - 2*n3*qy + 2*n2*qz;
X(4, 2) = 4*n1*qx + 2*n2*qy + 2*n3*qz;
X(4, 3) =           2*n2*qx - 2*n3*q0;
X(4, 4) =           2*n2*q0 + 2*n3*qx;

X(5, 1) = 4*n2*q0 + 2*n3*qx - 2*n1*qz;
X(5, 2) =           2*n3*q0 + 2*n1*qy;
X(5, 3) = 2*n1*qx + 4*n2*qy + 2*n3*qz;
X(5, 4) =           2*n3*qy - 2*n1*q0;

X(6, 1) = 4*n3*q0 - 2*n2*qx + 2*n1*qy;
X(6, 2) =           2*n1*qz - 2*n2*q0;
X(6, 3) =           2*n1*q0 + 2*n2*qz;
X(6, 4) = 2*n1*qx + 2*n2*qy + 4*n3*qz;

end