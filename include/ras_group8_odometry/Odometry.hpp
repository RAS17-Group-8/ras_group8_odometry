#ifndef RAS_GROUP8_ODOMETRY
#define RAS_GROUP8_ODOMETRY

// ROS
#include <ros/ros.h>

/* TODO: Document the methods.
 */

namespace ras_group8_odometry
{

class Odometry
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Odometry(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Odometry();

 private:
   
  bool readParameters();

  void leftWheelEncoderCallback();
  void rightWheelEncoderCallback();

  ros::NodeHandle& nodeHandle_;

  ros::Subscriber leftWheelEncoderSubscriber_;
  ros::Subscriber rightWheelEncoderSubscriber_;
  
  ros::Publisher odometryPublisher_;

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