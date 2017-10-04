#include <ros/ros.h>
#include "ras_group8_odometry/Odometry.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_odometry");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(10.0);
  ras_group8_odometry::Odometry odometry(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;
}
