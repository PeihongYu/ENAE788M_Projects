# Worked with my teammate Guangyao Shi.

import logging
logging.basicConfig()
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def main():
    bag_files = ["20211012_handheld.bag", "20211012_orientationctrl_test.bag", "20211012_positionctl_test.bag"]
    # for i in range(3):
    #     bag = rosbag.Bag(bag_files[0], "r")
    #     topics = bag.get_type_and_topic_info()[1].keys()
    #     print(topics)

    bag = rosbag.Bag(bag_files[1], "r")

    raw_data_topics = ['/imu0', '/imu1', '/mavros/imu/data_raw']
    gt_data_topics = ['/vicon/m500_joec/m500_joec', '/mavros/local_position/pose', '/mavros/imu/data']

    imu_data_topics = ['/imu0', '/imu1', '/mavros/imu/data_raw', '/mavros/imu/data']
    vicon_data_topic = '/vicon/m500_joec/m500_joec'
    mavros_data_topic = '/mavros/local_position/pose'

    for topic in imu_data_topics:
        f = open(topic.replace('/', '_')[1:] + '.txt', "a")
        for _, msg, t in bag.read_messages(topics=topic):
            orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            orientation_cov = msg.orientation_covariance
            angular_velocity = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
            angular_velocity_cov = msg.angular_velocity_covariance
            linear_acceleration = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
            linear_acceleration_cov = msg.linear_acceleration_covariance
            data = tuple([t.to_time()]) + orientation + orientation_cov + angular_velocity + angular_velocity_cov + linear_acceleration + linear_acceleration_cov
            f.write(str(data).replace('(','').replace(')', '') + '\n')
        f.close()

    topic = vicon_data_topic
    f = open(topic.replace('/', '_')[1:] + '.txt', "a")
    for _, msg, t in bag.read_messages(topics=topic):
        translation = (msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        rotation = (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
        data = tuple([t.to_time()]) + translation + rotation
        f.write(str(data).replace('(', '').replace(')', '') + '\n')
    f.close()

    topic = mavros_data_topic
    f = open(topic.replace('/', '_')[1:] + '.txt', "a")
    for _, msg, t in bag.read_messages(topics=topic):
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        orientation = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        data = tuple([t.to_time()]) + position + orientation
        f.write(str(data).replace('(', '').replace(')', '') + '\n')
    f.close()

    bag.close()

    return

if __name__ == '__main__':
    main()
