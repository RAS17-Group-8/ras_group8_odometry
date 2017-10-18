#ifndef RAS_GROUP8_ODOMETRY
#define RAS_GROUP8_ODOMETRY

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>

/* TODO: Document the methods.
 */

namespace ras_group8_odometry
{

#define ERROR_MISSING_PARAMETER 3
  
typedef struct {
  double x;
  double y;
  double theta;
} state_t;

class Odometry
{
 public:
  /* Constructor.
   * @param node_handle the ROS node handle.
   */
  Odometry(ros::NodeHandle& node_handle,
           double wheel_distance,
           const std::string& left_motor_twist_topic,
           const std::string& right_motor_twist_topic,
           const std::string& odometry_publish_topic,
           const std::string& frame_id,
           const std::string& child_frame_id);
  
  /* Destructor.
   */
  virtual ~Odometry();
  
  void
    update();
  
  static Odometry
    load(ros::NodeHandle& n);

 private:
  void
    leftMotorTwistCallback(const geometry_msgs::TwistStamped& msg);
    
  void
    rightMotorTwistCallback(const geometry_msgs::TwistStamped& msg);

  /* Main Node Handle
   */
  ros::NodeHandle& node_handle_;
  
  /* Subscribers
   */
  ros::Subscriber left_motor_twist_subscriber_;
  ros::Subscriber right_motor_twist_subscriber_;
  
  /* Publishers for the odometry and the reference frame
   */
  ros::Publisher odometry_publisher_;
  tf::TransformBroadcaster frame_broadcaster_;

  /* Parameters
   */
  const double wheel_distance_;
  
  /* Variables
   */
  geometry_msgs::TwistStamped left_motor_twist_msg;
  geometry_msgs::TwistStamped right_motor_twist_msg;
  
  ros::Time last_updated_time_;
  
  nav_msgs::Odometry odometry_msg_;
  geometry_msgs::TransformStamped odometry_transform_;
  
  state_t state_;
};

} /* namespace */

#endif /* RAS_GROUP8_ODOMETRY */
