// A ROS node that publishes radar CFAR detections as PointCloud messages.
// 1. Open radar sensor interface with mav_sensors.
// 2. While loop to read radar data at predefined rate.
// 3. RANSAC least squares fit to estimate linear velocity.
// 4. Publish radar detections as PointCloud messages.

#include <mav_sensors_core/sensor_config.h>
#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <log++.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/least_squares.h"
#include "rio/common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("velocity", 1000);
    ros::Publisher radar_pub = n.advertise<sensor_msgs::PointCloud>("cfar_detections", 1000);
    ros::Rate loop_rate(10);

    // Initialize radar sensor interface.
    SensorConfig cfg;
    cfg.set("path_cfg_file", ros::package::getPath("mav_sensors_demo") + "/cfg/radar/xwr18xx_AOP_profile_best_velocity_resolution.cfg");
    cfg.set("path_cfg", "/dev/ttyUSB0");
    cfg.set("path_data", "/dev/ttyUSB1");
    cfg.set("trigger", "false");

    Xwr18XxMmwDemo radar(cfg);
    if (!radar.open())
    {
        LOG(F, "Open failed.");
        return -1;
    }

    while (ros::ok())
    {
        auto measurement = radar.read();
        // RANSAC least squares fit to estimate linear velocity.
        Eigen::Vector3d velocity;
        if (rio::leastSquares(std::get<Radar>(measurement), &velocity))
        {
            LOG(D, "Velocity: " << velocity.transpose());

            // Publish least squares velocity estimate.
            geometry_msgs::Vector3Stamped msg_velocity;
            // Convert nanoseconds to seconds and remaining nanoseconds.
            msg_velocity.header.stamp = rio::toRosTime(std::get<Radar>(measurement).unix_stamp_ns);
            msg_velocity.header.frame_id = "awr1843aop";
            tf2::toMsg(velocity, msg_velocity.vector);
            vel_pub.publish(msg_velocity);
        }
        else
        {
            LOG(D, "Least squares failed.");
        }

        // Publish radar detections as PointCloud messages.
        sensor_msgs::PointCloud msg;
        msg.header.stamp = rio::toRosTime(std::get<Radar>(measurement).unix_stamp_ns);
        msg.header.frame_id = "awr1843aop";
        msg.points.resize(std::get<Radar>(measurement).cfar_detections.size());
        msg.channels.resize(3);
        msg.channels[0].name = "velocity";
        msg.channels[0].values.resize(std::get<Radar>(measurement).cfar_detections.size());
        msg.channels[1].name = "snr";
        msg.channels[1].values.resize(std::get<Radar>(measurement).cfar_detections.size());
        msg.channels[2].name = "noise";
        msg.channels[2].values.resize(std::get<Radar>(measurement).cfar_detections.size());
        for (size_t i = 0; i < std::get<Radar>(measurement).cfar_detections.size(); i++)
        {
            msg.points[i].x = std::get<Radar>(measurement).cfar_detections[i].x;
            msg.points[i].y = std::get<Radar>(measurement).cfar_detections[i].y;
            msg.points[i].z = std::get<Radar>(measurement).cfar_detections[i].z;
            msg.channels[0].values[i] = std::get<Radar>(measurement).cfar_detections[i].velocity;
            msg.channels[1].values[i] = std::get<Radar>(measurement).cfar_detections[i].snr;
            msg.channels[2].values[i] = std::get<Radar>(measurement).cfar_detections[i].noise;
        }
        radar_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    radar.close();
    return 0;
}