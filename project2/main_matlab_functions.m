clear;
addpath("algos_quaternion")

% Matlab: Choose Inertial Sensor Fusion Filters
% https://www.mathworks.com/help/fusion/ug/introduction-on-choosing-inertial-sensor-fusion-filters.html

folders = ["20211012_handheld", "20211012_orientationctrl_test", "20211012_positionctl_test"];
fid = 1;
raw_data_topics = ["imu0", "imu1", "mavros_imu_data_raw", "mavros_imu_mag"];
gt_data_topics = ["vicon_m500_joec_m500_joec", "mavros_local_position_pose", "mavros_imu_data"];

imu_data_raw = load(folders(fid) + "/" + raw_data_topics(3) + ".txt");
imu_mag_raw = load(folders(fid) + "/" + raw_data_topics(4) + ".txt");

t1 = imu_data_raw(:, 1) - imu_data_raw(1, 1);
t2 = imu_mag_raw(:, 1) - imu_data_raw(1, 1);

gyro = imu_data_raw(:, 15:17);    % Angular Velocity
accel = imu_data_raw(:, 27:29);   % Linear Accelaration
mag = imu_mag_raw(:, 2:4);        % Magnetometers
mag = interp1(t2, mag, t1, 'linear');    % Align mag with gyro and accel

sampleRate = 1/mean(t1(2:end)-t1(1:end-1));
% Run Matlab Complementary Filter
FUSE = complementaryFilter('SampleRate', sampleRate);
[orientation_CF, angularVelocity_CF] = FUSE(accel(1:end-1, :), gyro(1:end-1, :), mag(1:end-1, :));

% Run Matlab Kalman Filter
FUSE = ahrsfilter('SampleRate', sampleRate);
[orientation_KF, angularVelocity_KF] = FUSE(accel(1:end-1, :), gyro(1:end-1, :), mag(1:end-1, :));

% load gt
imu_data_gt = load(folders(fid) + "/" + gt_data_topics(3) + ".txt");
imu_data_quat = imu_data_gt(:, [5, 2:4]);
t2 = imu_data_gt(:, 1) - imu_data_raw(1, 1);

% translater to euler angle
type = "euler";
gt_eul = normalize_orientation(imu_data_quat, type);
res_eul_CF = normalize_orientation(orientation_CF, type);
res_eul_KF = normalize_orientation(orientation_KF, type);

figure;
titles = ["yaw", "pitch", "roll"];
for i = 1:3
    subplot(3, 1, i);
    plot(t2, gt_eul(:, i));
    hold on;
    plot(t1(1:end-1,:), res_eul_CF(:, i));
    hold on;
    plot(t1(1:end-1,:), res_eul_KF(:, i));
    title(titles(i))
    legend("mavros/imu/data", "complementary filter", "kalman filter")
end

