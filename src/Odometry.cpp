#include <ras_group8_odometry/Odometry.hpp>

#include <math.h>

namespace ras_group8_odometry {

/* Constructor
 */
Odometry::Odometry(ros::NodeHandle& node_handle,
                   double wheel_distance,
                   const std::string& left_motor_twist_topic,
                   const std::string& right_motor_twist_topic,
                   const std::string& odometry_publish_topic,
                   const std::string& frame_id,
                   const std::string& child_frame_id)
    : node_handle_(node_handle),
      wheel_distance_(wheel_distance),
      last_updated_time_(ros::Time::now())
{
  /* Set the state to a known inital position */
  state_.x     = 0;
  state_.y     = 0;
  state_.theta = 0;
  state_.v     = 0;
  state_.w     = 0;
  
  /* Setup constant fields in messages */
  odometry_msg_.header.frame_id       = frame_id;
  odometry_msg_.child_frame_id        = child_frame_id;
  odometry_msg_.pose.pose.position.z  = 0.0;
  
  odometry_transform_.header.frame_id = frame_id;
  odometry_transform_.child_frame_id  = child_frame_id;
  odometry_transform_.transform.translation.z = 0.0;
  
  /* Setup publishers and subscribers */
  left_motor_twist_subscriber_ =
    node_handle_.subscribe(left_motor_twist_topic, 10,
                           &Odometry::leftMotorTwistCallback, this);
  
  right_motor_twist_subscriber_ =
    node_handle_.subscribe(right_motor_twist_topic, 10,
                           &Odometry::rightMotorTwistCallback, this);
                           
  odometry_publisher_=
    node_handle_.advertise<nav_msgs::Odometry>(odometry_publish_topic, 10);
    
  ROS_INFO("Successfully launched node.");
}

/* Destructor
 */
Odometry::~Odometry()
{
}

/* Update State
 * Private method that is called whenever the internal state needs to be
 * updated.
 */
void Odometry::updateState()
{
  ros::Time t0;
  ros::Time t1;
  
  double dt;
  double v_left;
  double v_right;
  
  bool valid_left  = left_motor_twist_msgs.isValid();
  bool valid_right = right_motor_twist_msgs.isValid();
  
  if (!valid_left || !valid_right) {
    if (valid_left) {
      last_updated_time_ = right_motor_twist_msgs.head()->header.stamp;
    } else if (valid_right) {
      last_updated_time_ = left_motor_twist_msgs.head()->header.stamp;
    } else {
      ROS_INFO("Motor msg buffers are warming up");
    }
    return;
  }
      
  /* Set t0 */
  if (left_motor_twist_msgs.tail()->header.stamp <
                                  right_motor_twist_msgs.tail()->header.stamp) {
    t0 = right_motor_twist_msgs.tail()->header.stamp;
  } else {
    t0 = left_motor_twist_msgs.tail()->header.stamp;
  }
  
  /* Set t1 */
  if (left_motor_twist_msgs.head()->header.stamp <
                                  right_motor_twist_msgs.head()->header.stamp) {
    t1 = left_motor_twist_msgs.head()->header.stamp;
  } else {
    t1 = right_motor_twist_msgs.head()->header.stamp;
  }
  
  /* If not then we missed something */
  ROS_ASSERT(last_updated_time_ == t0);
  
  /* Extract the valid region of time t1 - t0
   * We know for sure that the distance traveled during this
   * time is dt*v_{left,right}.
   */
  dt      = (t1 - t0).toSec();
  v_left  = left_motor_twist_msgs.head()->twist.linear.x;
  v_right = right_motor_twist_msgs.head()->twist.linear.x;
  
  state_.v = (v_right + v_left) / 2;
  state_.w = atan2((v_right - v_left), wheel_distance_);
  
  state_.x     += state_.v * cos(state_.w) * dt;
  state_.y     += state_.v * sin(state_.w) * dt;
  state_.theta += state_.w * dt;
  
  last_updated_time_ = t1;
}

/* Left Motor Twist Callback
 */
void Odometry::leftMotorTwistCallback(const geometry_msgs::TwistStamped& msg)
{
  left_motor_twist_msgs.insert(msg);
  updateState();
}

/* Right Motor Twist Callback
 */
void Odometry::rightMotorTwistCallback(const geometry_msgs::TwistStamped& msg)
{
  right_motor_twist_msgs.insert(msg);
  updateState();
}

/* Update
 */
void Odometry::update()
{
  odometry_msg_.header.stamp = last_updated_time_;
  
  geometry_msgs::Quaternion odom_quat =
    tf::createQuaternionMsgFromYaw(state_.theta);
  
  /* Set the position */
  odometry_msg_.pose.pose.position.x  = state_.x;
  odometry_msg_.pose.pose.position.y  = state_.y;
  odometry_msg_.pose.pose.orientation = odom_quat;
  
  /* set the velocity */
  odometry_msg_.twist.twist.linear.x = state_.v;
  odometry_msg_.twist.twist.angular.z = state_.w;
    
  /* Publish the odometry */
  odometry_publisher_.publish(odometry_msg_);
  
  /* Publish the frame */
  odometry_transform_.header.stamp = last_updated_time_;
    
  odometry_transform_.transform.translation.x = state_.x;
  odometry_transform_.transform.translation.y = state_.y;
  odometry_transform_.transform.rotation      = odom_quat;
  
  frame_broadcaster_.sendTransform(odometry_transform_);
}

/* Load
 */
Odometry Odometry::load(ros::NodeHandle& n)
{
  double wheel_distance;
  std::string left_motor_twist_topic;
  std::string right_motor_twist_topic;
  std::string odometry_topic;
  std::string frame_id;
  std::string child_frame_id;
  
  /* Load required parameters */
  if (!n.getParam("/platform/wheel_distance", wheel_distance))
    exit(ERROR_MISSING_PARAMETER);
  ROS_INFO("wheel_distance = %f", wheel_distance);
  
  if (!n.getParam("left_motor_twist_topic", left_motor_twist_topic))
    exit(ERROR_MISSING_PARAMETER);
  ROS_INFO("left_motor_twist_topic = %s", left_motor_twist_topic.c_str());
  
  if (!n.getParam("right_motor_twist_topic", right_motor_twist_topic))
    exit(ERROR_MISSING_PARAMETER);
  ROS_INFO("right_motor_twist_topic = %s", right_motor_twist_topic.c_str());
      
  /* Load optional parameters */
  odometry_topic = n.param("odometry_topic", std::string("odometry"));
  frame_id       = n.param("frame_id", std::string("odom"));
  child_frame_id = n.param("child_frame_id", std::string("base_link"));
  
  Odometry odometry(n, wheel_distance,
                       left_motor_twist_topic,
                       right_motor_twist_topic,
                       odometry_topic,
                       frame_id,
                       child_frame_id);
  
  return odometry;
}

} /* namespace */
