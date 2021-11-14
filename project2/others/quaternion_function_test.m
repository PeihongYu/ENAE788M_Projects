% Matlab quaternion document
% https://www.mathworks.com/help/robotics/ref/quaternion.html

% quaternion multiplication
% https://www.mathworks.com/help/robotics/ref/quaternion.mtimes.html
p = rand(1, 4);
p = p / norm(p);
q = rand(1, 4);
q = q / norm(q);

q_mat = [q(1), -q(2), -q(3), -q(4);
         q(2), q(1), -q(4), q(3);
         q(3), q(4), q(1), -q(2);
         q(4), -q(3), q(2), q(1)];
     
res = (q_mat * p')'
res = (quaternion(q) * quaternion(p));
res.compact

% rotate vecter by quaternion
% https://www.mathworks.com/help/aerotbx/ug/quatrotate.html
v = rand(3, 1);
q_mat = [2 * q(1)^2 + 2 * q(2)^2 - 1, 2 * q(2) * q(3) - 2 * q(1) * q(4), 2 * q(2) * q(4) + 2 * q(1) * q(3);
        2 * q(2) * q(3) + 2 * q(1) * q(4), 2 * q(1)^2 + 2 * q(3)^2 - 1, 2 * q(3) * q(4) - 2 * q(1) * q(2);
        2 * q(2) * q(4) - 2 * q(1) * q(3), 2 * q(3) * q(4) + 2 * q(1) * q(2), 2 * q(1)^2 + 2 * q(4)^2 - 1];
res = (q_mat' * v)'

res = quatrotate(q, v')