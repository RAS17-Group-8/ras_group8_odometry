#include "ras_group8_odometry/Odometry.hpp"

// STD
#include <string>
#include <math.h>

/* Since the two encoder messages might not be published at the same time we
 * probably need to cache the response of the first one, whichever it may be.
 * Depending on how out of sync they are we might need some form of prediction
 * model.
 */

namespace ras_group8_odometry {

Odometry::Odometry(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!reload()) {
    ros::requestShutdown();
  }
  x=0;
  y=0;
  th=0;
  vx=0;
  vy=0;
  vth=0;
  vleft=0;
  vright=0;
  wheel_circumference=0.22934;
  odometryPublisher_= nodeHandle_.advertise<nav_msgs::Odometry>(publishTopic_, 50);
  //frameBroadcaster_=
  lasttime=ros::Time::now();
  timeleft=lasttime;
  timeright=lasttime;
  /* Setup the reload service
   */
  reloadService_ =
    nodeHandle_.advertiseService("reload", &Odometry::reloadCallback, this);
  
  ROS_INFO("Successfully launched node.");
}

Odometry::~Odometry()
{
}

bool Odometry::reload()
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    return false;
  }
  
  if (leftWheelEncoderSubscriber_) {
    leftWheelEncoderSubscriber_.shutdown();
  }
  
  if (rightWheelEncoderSubscriber_) {
    rightWheelEncoderSubscriber_.shutdown();
  }
  
  /* Subsribe to encoder updates */
  leftWheelEncoderSubscriber_ =
    nodeHandle_.subscribe(leftWheelEncoderTopic_, 1,
                          &Odometry::leftWheelEncoderCallback, this);
                          
  rightWheelEncoderSubscriber_ =
    nodeHandle_.subscribe(rightWheelEncoderTopic_, 1,
                          &Odometry::rightWheelEncoderCallback, this);
}

void Odometry::leftWheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  ros::Time time = ros::Time::now();
  double dt = (time-timeleft).toSec();
  timeleft=time;
  lasttime=time;

  vleft = (msg.count_change) * wheel_circumference /
              (encoderTicsPerRevolutionleft_) / dt;


  //ROS_INFO("time is %f",dt);
  ROS_INFO("find left wheel in v=%f",vleft);
  Odometry::publishOdometry(dt);
}

void Odometry::rightWheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  ros::Time time = ros::Time::now();
  double dt = (time-timeright).toSec();
  timeright=time;
  lasttime=time;

  


  vright = (-msg.count_change) * wheel_circumference /
              (encoderTicsPerRevolutionright_) / dt;

  ROS_INFO("find right wheel in v=%f",vright);

  Odometry::publishOdometry(dt);


}

void Odometry::publishOdometry(double dt)
{


    double v_left=vleft;
    double v_right=vright;

    vx = ((v_right + v_left) / 2);
    vy = 0;
    vth = -atan((v_right - v_left)/wheelDistance_);

    

    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;


    ROS_INFO("dx is %f",delta_x);
    ROS_INFO("vth is %f",vth);

    x += delta_x;
    y += delta_y;
    th += delta_th;


  nav_msgs::Odometry odometry;
  odometry.header.stamp = lasttime;
  odometry.header.frame_id = headerFrameId_;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  
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

  odometryPublisher_.publish(odometry);
  //ROS_INFO("publishing to topic ");

  Odometry::broadcastFrame(odom_quat);
}

void Odometry::broadcastFrame(geometry_msgs::Quaternion& quat)
{
  geometry_msgs::TransformStamped odometryTransform;
  
  odometryTransform.header.stamp = lasttime;
  odometryTransform.header.frame_id = headerFrameId_;
  odometryTransform.child_frame_id = childFrameId_;
  
  odometryTransform.transform.translation.x = x;
  odometryTransform.transform.translation.y = y;
  odometryTransform.transform.translation.z = 0.0;
  odometryTransform.transform.rotation = quat;
  
  frameBroadcaster_.sendTransform(odometryTransform);
}

bool Odometry::reloadCallback(std_srvs::Trigger::Request& request,
                                                std_srvs::Trigger::Response& response)
{
  if (reload()) {
    response.success = true;
  } else {
    response.success = false;
    response.message = "Failed to reload node";
  }
  
  return true;
}

bool Odometry::readParameters()
{
  /* Try to load all the parameters. Return false if any one
     of them fails. */
  if (!nodeHandle_.getParam("wheel_distance",
                            wheelDistance_))
    return false;
  ROS_INFO("Using wheel distance %f", wheelDistance_);
  
  if (!nodeHandle_.getParam("wheel_radius",
                            wheelRadius_))
    return false;
  ROS_INFO("Using wheel radius %f", wheelRadius_);
  
  if (!nodeHandle_.getParam("left_wheel_encoder_topic",
                            leftWheelEncoderTopic_))
    return false;
  //ROS_INFO("left topic is %s",leftWheelEncoderTopic_.c_str());
  if (!nodeHandle_.getParam("right_wheel_encoder_topic",
                            rightWheelEncoderTopic_))
    return false;
  if (!nodeHandle_.getParam("header_frame_id",
                            headerFrameId_))
    return false;
  if (!nodeHandle_.getParam("child_frame_id",
                            childFrameId_))
    return false;

  if (!nodeHandle_.getParam("encoder_ticks_per_rev_left",
                            encoderTicsPerRevolutionleft_))
    return false;

  if (!nodeHandle_.getParam("encoder_ticks_per_rev_right",
                            encoderTicsPerRevolutionright_))
    return false;
  


   
  return true;
}

} /* namespace */
