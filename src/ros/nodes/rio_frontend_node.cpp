#include <ros/ros.h>

#include "rio/ros/rio_frontend.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rio_frontend_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rio::RioFrontend rio_frontend(nh, nh_private);
  if (!rio_frontend.init()) return 1;

  ros::spin();
  return 0;
}