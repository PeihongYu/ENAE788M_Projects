clear;
close all;

folders = ["20211012_handheld", "20211012_orientationctrl_test", "20211012_positionctl_test"];
fid = 1;
raw_data_topics = ["imu0", "imu1", "mavros_imu_data_raw", "mavros_imu_mag"];
gt_data_topics = ["vicon_m500_joec_m500_joec", "mavros_local_position_pose", "mavros_imu_data"];
imu_data_topics = ["imu0", "imu1", "mavros_imu_data_raw", "mavros_imu_data"];

% plot imu data
imu0 = load("../" + folders(fid) + "/imu0.txt");
figure;
for i = 1: 2
    imu = load("../" + folders(fid) + "/" + imu_data_topics(i) + ".txt");
    t = imu(:, 1) - imu0(1, 1);
    data1 = imu(:, 15:17);
    data2 = imu(:, 27:29);
    flag = t<50 & t>10;
    names = ["x", "y", "z"];
    for j = 1:3
        subplot(2,3,j)
        hold on;
        plot(t(flag), data1(flag, j))
        title("angular velocity " + names(j))
    end
    for j = 1:3
        subplot(2,3,j+3);
        hold on;
        plot(t(flag), data2(flag, j))
        title("linear accelaration " + names(j))
    end
end