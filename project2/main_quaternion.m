clear;
addpath("algos_quaternion")

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

% Run Complementary Filter
orientation = complementary_filter(accel, gyro, mag, t1);

% Run Kalman Filter
% orientation = kalman_filter(accel, gyro, mag, t1);

% Run Extended Kalman Filter
% orientation = extended_kalman_filter(accel, gyro, mag, t1);

% load gt
imu_data_gt = load(folders(fid) + "/" + gt_data_topics(3) + ".txt");
imu_data_quat = imu_data_gt(:, [5, 2:4]);
t2 = imu_data_gt(:, 1) - imu_data_raw(1, 1);
vicon_gt = load(folders(fid) + "/vicon_m500_joec_m500_joec.txt");
vicon_quat = vicon_gt(:, [6:8, 5]);
t3 = vicon_gt(:, 1) - imu_data_raw(1, 1);

type = "euler";
gt_eul = normalize_orientation(imu_data_quat, type);
res_eul = normalize_orientation(orientation, type);
vicon_eul = normalize_orientation(vicon_quat, type);

figure;
titles = ["yaw", "pitch", "roll"];
for i = 1:3
    subplot(3, 1, i)
    % plot(t3, vicon_eul(:, i))
    % hold on
    plot(t2, gt_eul(:, i))
    hold on
    plot(t1, res_eul(:, i))
    title(titles(i))
    legend("mavros/imu/data", "estimation")
end
