#include "ras_group8_odometry/Odometry.hpp"

// STD
#include <string>

/* Since the two encoder messages might not be published at the same time we
 * probably need to cache the response of the first one, whichever it may be.
 * Depending on how out of sync they are we might need some form of prediction
 * model.
 */

namespace ras_group8_odometry {

Odometry::Odometry(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  /* Setup the reload service
   */
  reloadService_ =
    nodeHandle_.advertiseService("reload", &Odometry::reloadCallback, this);
  
  /* Subsribe to encoder updates */
  leftWheelEncoderSubscriber_ =
    nodeHandle_.subscribe(leftWheelEncoderTopic_, 1,
                          &Odometry::leftWheelEncoderCallback, this);
                          
  rightWheelEncoderSubscriber_ =
    nodeHandle_.subscribe(rightWheelEncoderTopic_, 1,
                          &Odometry::rightWheelEncoderCallback, this);
  
  ROS_INFO("Successfully launched node.");
}

Odometry::~Odometry()
{
}

void Odometry::leftWheelEncoderCallback(const phidgets::motor_encoder& msg)
{
    double velocity;
    double time = ros::Time::now().toSec();
    double dt;


    vleft = msg.count_change * wheel_circumference /
                encoderTicsPerRevolution_ / dt;
    double v_left=vleft;
    double v_right=vright;

    vx = ((v_right + v_left) / 2)*10;
    vy = 0;
    vth = ((v_right - v_left)/lengthBetweenTwoWheels)*10;
    double dt=time-lasttime;
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    Odometry::publishOdometry();
}

void Odometry::rightWheelEncoderCallback(const phidgets::motor_encoder& msg)
{
    double velocity;
    double time = ros::Time::now().toSec();
    double dt;


    vright = msg.count_change * wheel_circumference /
                encoderTicsPerRevolution_ / dt;

    double v_left=vleft;
    double v_right=vright;

    vx = ((v_right + v_left) / 2)*10;
    vy = 0;
    vth = ((v_right - v_left)/lengthBetweenTwoWheels)*10;
    double dt=time-lasttime;
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    Odometry::publishOdometry();


}

void publishOdometry()
{





  nav_msgs::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = headerFrameId_;
  
  //set the position
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = odom_quat;
  
  //set the velocity
  odometry.child_frame_id = childFrameId_;
  odometry.twist.twist.linear.x = vx;
  odometry.twist.twist.linear.y = vy;
  odometry.twist.twist.angular.z = vth;
  
  //publish the message
  odom_pub.publish(odom);
}

void broadcastFrame(double x, double y, geometry_msgs::Quaternion& quat)
{
  geometry_msgs::TransformStamped odometryTransform;
  
  odometryTransform.header.stamp = current_time;
  odometryTransform.header.frame_id = headerFrameId_;
  odometryTransform.child_frame_id = childFrameId_;
  
  odometryTransform.transform.translation.x = x;
  odometryTransform.transform.translation.y = y;
  odometryTransform.transform.translation.z = 0.0;
  odometryTransform.transform.rotation = quat;
  
  odometryPublisher_.sendTransform(odometryTransform);
}

bool Odometry::reloadCallback(std_srvs::Trigger::Request& request,
                                                std_srvs::Trigger::Response& response)
{
  if (readParameters()) {
    response.success = true;
  } else {
    response.success = false;
    response.message = "Failed to reload parameters";
  }
  
  return true;
}

bool Odometry::readParameters()
{
  /* Try to load all the parameters. Return false if any one
     of them fails. */
  if (!nodeHandle_.getParam("/ras_group8_platform/wheel_distance",
                            wheelDistance_))
    return false;
  ROS_INFO("Using wheel distance %f", wheelDistance_);
  
  if (!nodeHandle_.getParam("/ras_group8_platform/wheel_radius",
                            wheelRadius_))
    return false;
  ROS_INFO("Using wheel radius %f", wheelRadius_);
  
  if (!nodeHandle_.getParam("left_wheel_encoder_topic",
                            leftWheelEncoderTopic_))
    return false;
  if (!nodeHandle_.getParam("right_wheel_encoder_topic",
                            rightWheelEncoderTopic_))
    return false;
  if (!nodeHandle_.getParam("publish_topic",
                            publishTopic_))
    return false;
  
  return true;
}

} /* namespace */
