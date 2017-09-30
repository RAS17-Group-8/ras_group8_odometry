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
}

void Odometry::rightWheelEncoderCallback(const phidgets::motor_encoder& msg)
{
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
                            wheelBase_))
    return false;
  ROS_INFO("Using wheel distance %f", wheelBase_);
  
  if (!nodeHandle_.getParam("/ras_group8_platform/wheel_radius",
                            wheelRadius_))
    return false;
  ROS_INFO("Using wheel radius %f", wheelBase_);
  
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