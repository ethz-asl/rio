/*
 * This node loads a rosbag with IMU and radar data, sets up a calibration
 * optimization and solves for the states and extrinsic calibration.
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "rio/gtsam/expressions.h"
#include "rio/gtsam/optimization.h"

using namespace rio;
using namespace gtsam;
using namespace ros;

int main(int argc, char** argv) {
  init(argc, argv, "rio_calibration_node");
  NodeHandle nh;
  NodeHandle nh_private("~");

  // Load parameters.
  std::string bag_path;
  nh_private.param<std::string>("bag_path", bag_path, "");

  // Load rosbag.
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Read IMU and radar data from rosbag
}