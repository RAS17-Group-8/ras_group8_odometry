#include <ros/ros.h>
#include "ras_group8_odometry/Odometry.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_odometry");
  ros::NodeHandle nodeHandle("~");

  ras_group8_odometry::Odometry odometry(nodeHandle);

  ros::spin();
  return 0;
}