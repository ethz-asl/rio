#include <ros/ros.h>

#include "rio/ros/rio.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rio_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rio::Rio rio(nh, nh_private);
  if (!rio.init()) return 1;

  ros::spin();
  return 0;
}