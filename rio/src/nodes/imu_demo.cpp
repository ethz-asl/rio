// A ROS node that publishes imu measurements.
// 1. Open imu sensor interface with mav_sensors.
// 2. While loop to read imu data at predefined rate.
// 3. Publish imu as imu messages.

#include <mav_sensors_core/sensor_config.h>
#include <mav_sensors_drivers/imu/bmi088.h>
#include <log++.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "rio/common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_demo");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_raw", 1000);
    ros::Rate loop_rate(200);

    // Initialize imu sensor interface.
    SensorConfig cfg;
    cfg.set("path_acc", "/dev/spidev0.0");
    cfg.set("path_gyro", "/dev/spidev0.1");

    Bmi088<Spi> bmi088(cfg);
    if (!bmi088.open())
    {
        LOG(F, "Open failed");
        return 1;
    }

    while (ros::ok())
    {
        auto measurement = bmi088.read();

        if (std::get<0>(measurement).has_value() && std::get<1>(measurement).has_value() &&std::get<2>(measurement).has_value())
        {
            sensor_msgs::Imu msg;
            msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
            msg.header.frame_id = "bmi088";
            msg.angular_velocity.x = std::get<1>(measurement).value().x;
            msg.angular_velocity.y = std::get<1>(measurement).value().y;
            msg.angular_velocity.z = std::get<1>(measurement).value().z;
            msg.linear_acceleration.x = std::get<0>(measurement).value().x;
            msg.linear_acceleration.y = std::get<0>(measurement).value().y;
            msg.linear_acceleration.z = std::get<0>(measurement).value().z;
            msg.orientation_covariance[0] = -1; // orientation not supported by bmi088
            imu_pub.publish(msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    bmi088.close();
    return 0;
}