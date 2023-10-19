/*
 * This node loads a rosbag with IMU and radar data, sets up a calibration
 * optimization and solves for the states and extrinsic calibration.
 */

#include <gtsam/geometry/Pose3.h>
#include <log++.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/gtsam/expressions.h"
#include "rio/gtsam/optimization.h"
#include "rio/ros/common.h"

using namespace rio;
using namespace gtsam;
using namespace ros;

struct ImuMeasurement {
  double t;
  Vector3 a;
  Vector3 w;
};

struct RadarMeasurement {
  double t;
  Vector3 p;
  double v;
};

struct OrientationMeasurement {
  double t;
  Rot3 R_IB;
};

int main(int argc, char** argv) {
  init(argc, argv, "rio_calibration_node");
  NodeHandle nh;
  NodeHandle nh_private("~");

  // Load parameters.
  std::string bag_path;
  if (!loadParam<std::string>(nh_private, "bag_path", &bag_path)) {
    return 1;
  }
  std::string orientation_topic;
  if (!loadParam<std::string>(nh_private, "orientation_topic",
                              &orientation_topic)) {
    return 1;
  }
  std::string imu_topic;
  if (!loadParam<std::string>(nh_private, "imu_topic", &imu_topic)) {
    return 1;
  }
  std::string radar_topic;
  if (!loadParam<std::string>(nh_private, "radar_topic", &radar_topic)) {
    return 1;
  }

  // Load rosbag.
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Read IMU, IMU raw and radar data from rosbag
  std::vector<OrientationMeasurement> orientation_measurements;
  std::vector<ImuMeasurement> imu_raw_measurements;
  std::vector<RadarMeasurement> radar_measurements;

  rosbag::View view(bag);
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.getTopic() == orientation_topic) {
      sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      Eigen::Quaterniond q_IB;
      tf2::fromMsg(imu_msg->orientation, q_IB);
      OrientationMeasurement orientation_measurement;
      orientation_measurement.t = imu_msg->header.stamp.toSec();
      orientation_measurement.R_IB = Rot3(q_IB);
      orientation_measurements.push_back(orientation_measurement);
    } else if (msg.getTopic() == imu_topic) {
      sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      ImuMeasurement imu_measurement;
      imu_measurement.t = imu_msg->header.stamp.toSec();
      tf2::fromMsg(imu_msg->linear_acceleration, imu_measurement.a);
      tf2::fromMsg(imu_msg->angular_velocity, imu_measurement.w);
      imu_raw_measurements.push_back(imu_measurement);
    } else if (msg.getTopic() == radar_topic) {
      sensor_msgs::PointCloud2Ptr radar_msg =
          msg.instantiate<sensor_msgs::PointCloud2>();
      auto detections = parseRadarMsg(radar_msg);
      for (const auto& detection : detections) {
        RadarMeasurement radar_measurement;
        radar_measurement.t = radar_msg->header.stamp.toSec();
        radar_measurement.p = {detection.x, detection.y, detection.z};
        if (radar_measurement.p.norm() < 0.1) {
          LOG(W, "Ignoring radar measurement with detection distance "
                     << radar_measurement.p.norm());
          continue;
        }
        radar_measurement.v = detection.velocity;
        radar_measurements.push_back(radar_measurement);
      }
    }
  }

  LOG(I, "Loaded " << orientation_measurements.size()
                   << " orientation measurements, "
                   << imu_raw_measurements.size() << " IMU measurements and "
                   << radar_measurements.size() << " radar measurements.");

  return 0;
}