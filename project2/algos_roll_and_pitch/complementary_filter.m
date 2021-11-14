function angles = complementary_filter(accel, gyro, time)

phi   = zeros(length(time), 1);     % Roll
theta = zeros(length(time), 1);     % Pitch

alpha = 0.7;

for t = 2:length(time)
    p = gyro(t, 1) * pi / 180;
    q = gyro(t, 2) * pi / 180;
    r = gyro(t, 3) * pi / 180;
    
    phi_pre = phi(t-1);
    theta_pre = phi(t-1);
    
    phi_acc   = atan2( accel(t, 2), sqrt(accel(t, 1) .^ 2 + accel(t, 3) .^ 2));
    theta_acc = atan2(-accel(t, 1), sqrt(accel(t, 2) .^ 2 + accel(t, 3) .^ 2));
    
    delta_t = time(t) - time(t-1);
    phi_gyro   = phi_pre   + delta_t * (p + sin(phi_pre) * tan(theta_pre) * q + cos(phi_pre) * tan(theta_pre) * r);
    theta_gyro = theta_pre + delta_t * (cos(phi_pre) * q - sin(phi_pre) * r);
    
    phi(t)   = (1 - alpha) * phi_gyro   + alpha * phi_acc;
    theta(t) = (1 - alpha) * theta_gyro + alpha * theta_acc;  
    
end

angles = [theta, phi];

end