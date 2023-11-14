#!/usr/bin/python3

# import rosbag, go through IMU messages and offset them by a constant

# Open rosbag
import rosbag

# Open bag
bag_path = '/home/brik/Desktop/2023-10-31-10-39-08'
bag_in = rosbag.Bag(bag_path + '.bag', 'r')

# Open bag for writing
bag_out = rosbag.Bag(bag_path + '_bias_imu.bag', 'w')

# Get IMU topic
topics = bag_in.get_type_and_topic_info()[1].keys()
print(topics)
imu_topic = '/imu/data_raw'

for topic, msg, t in bag_in.read_messages():
    if(topic == imu_topic):
        msg.linear_acceleration.x += 0.1
        msg.linear_acceleration.y += 0.1
        msg.linear_acceleration.z += 0.1
        msg.angular_velocity.x += 0.1
        msg.angular_velocity.y += 0.1
        msg.angular_velocity.z += 0.1
    bag_out.write(topic, msg, t)

bag_in.close()
bag_out.close()

