clear;
addpath("algos_roll_and_pitch")

folders = ["20211012_handheld", "20211012_orientationctrl_test", "20211012_positionctl_test"];
fid = 1;
raw_data_topics = ["imu0", "imu1", "mavros_imu_data_raw", "mavros_imu_mag"];
gt_data_topics = ["vicon_m500_joec_m500_joec", "mavros_local_position_pose", "mavros_imu_data"];

imu_data_raw = load(folders(fid) + "/" + raw_data_topics(3) + ".txt");
t1 = imu_data_raw(:, 1) - imu_data_raw(1, 1);

gyro = imu_data_raw(:, 15:17);    % Angular Velocity
accel = imu_data_raw(:, 27:29);   % Linear Accelaration

% Run Complementary Filter
angles_CF = complementary_filter(accel, gyro, t1);

% Run Kalman Filter
angles_KF = kalman_filter(accel, gyro, t1);

% load gt
imu_data_gt = load(folders(fid) + "/" + gt_data_topics(3) + ".txt");
imu_data_quat = imu_data_gt(:, [5, 2:4]);
t2 = imu_data_gt(:, 1) - imu_data_raw(1, 1);
vicon_gt = load(folders(fid) + "/vicon_m500_joec_m500_joec.txt");
vicon_quat = vicon_gt(:, [6:8, 5]);
t3 = vicon_gt(:, 1) - imu_data_raw(1, 1);

type = "euler";
gt_eul = normalize_orientation(imu_data_quat, type);
vicon_eul = normalize_orientation(vicon_quat, type);

figure;
titles = ["pitch", "roll"];
for i = 1:2
    subplot(2, 1, i)
    plot(t2, gt_eul(:, i + 1))
    hold on
    plot(t1, angles_CF(:, i))
    hold on
    plot(t1, angles_KF(:, i))
    title(titles(i))
    legend("mavros/imu/data", "complementary filter", "kalman filter")
end