#include <ros/ros.h>
#include <ras_group8_odometry/Odometry.hpp>

using namespace ras_group8_odometry;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_odometry");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  Odometry odometry = Odometry::load(node_handle);

  while (ros::ok()) {
    ros::spinOnce();
    odometry.update();
    loop_rate.sleep();
  }
  
  return 0;
}
