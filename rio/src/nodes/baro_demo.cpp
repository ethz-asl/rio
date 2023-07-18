// A ROS node that publishes baro pressure measurements.
// 1. Open baro sensor interface with mav_sensors.
// 2. While loop to read baro data at predefined rate.
// 3. Publish baro as fluid pressure messages.

#include <mav_sensors_core/sensor_config.h>
#include <mav_sensors_drivers/barometer/bmp390.h>
#include <log++.h>

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include "rio/common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baro_demo");
    ros::NodeHandle n;
    ros::Publisher baro_pub = n.advertise<sensor_msgs::FluidPressure>("pressure", 1000);
    ros::Publisher temp_pub = n.advertise<sensor_msgs::Temperature>("temperature", 1000);
    ros::Rate loop_rate(25);

    // Initialize baro sensor interface.
    SensorConfig cfg;
    cfg.set("path", "/dev/spidev2.0");

    BMP390<Spi> bmp390(cfg);
    if (!bmp390.open())
    {
        LOG(F, "Open failed");
        return 1;
    }

    while (ros::ok())
    {
        auto measurement = bmp390.read();

        if (std::get<0>(measurement).has_value() && std::get<2>(measurement).has_value())
        {
            // Publish pressure measurements.
            sensor_msgs::FluidPressure msg;
            msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
            msg.header.frame_id = "bmp390";
            msg.fluid_pressure = std::get<0>(measurement).value();
            baro_pub.publish(msg);
        }
        if (std::get<1>(measurement).has_value() && std::get<2>(measurement).has_value())
        {
            // Publish temperature measurements.
            sensor_msgs::Temperature msg;
            msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
            msg.header.frame_id = "bmp390";
            msg.temperature = std::get<1>(measurement).value();
            temp_pub.publish(msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    bmp390.close();
    return 0;
}