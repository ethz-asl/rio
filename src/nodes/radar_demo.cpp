// A ROS node that publishes radar CFAR detections as PointCloud messages.
// 1. Open radar sensor interface with mav_sensors.
// 2. While loop to read radar data at predefined rate.
// 3. Publish radar detections as PointCloud messages.

#include <mav_sensors_core/sensor_config.h>
#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <log++.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
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

    int count=0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        //std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        //msg.data = ss.str();

        ROS_INFO("loop");

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        //chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}