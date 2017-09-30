#ifndef RAS_GROUP8_ODOMETRY
#define RAS_GROUP8_ODOMETRY

// ROS
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <phidgets/motor_encoder.h>

/* TODO: Document the methods.
 */

namespace ras_group8_odometry
{

class Odometry
{
 public:
  /* Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Odometry(ros::NodeHandle& nodeHandle);
  
  /* Destructor.
   */
  virtual ~Odometry();

  /* Call this method to force the node to reload all of its parameter settings.
   * More commonly this would also be attached to a message callback.
   */
  bool reload();

 private:
   
  bool readParameters();

  void leftWheelEncoderCallback(const phidgets::motor_encoder& msg);
  void rightWheelEncoderCallback(const phidgets::motor_encoder& msg);
  
  bool reloadCallback(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);

  ros::NodeHandle& nodeHandle_;

  ros::Subscriber leftWheelEncoderSubscriber_;
  ros::Subscriber rightWheelEncoderSubscriber_;
  
  ros::Publisher odometryPublisher_;
  
  /* Services
   */
  ros::ServiceServer reloadService_;

  /* Parameters
   */
  std::string leftWheelEncoderTopic_;
  std::string rightWheelEncoderTopic_;
  std::string publishTopic_;
  
  double wheelBase_;
  double wheelRadius_;
  
};

} /* namespace */

#endif /* RAS_GROUP8_ODOMETRY */