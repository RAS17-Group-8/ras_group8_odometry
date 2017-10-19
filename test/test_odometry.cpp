#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <ras_group8_odometry/Odometry.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

using namespace ras_group8_odometry;

static nav_msgs::Odometry last_odometry_msg;
static std::string last_odometry_msg_frame_id;
static std::string last_odometry_msg_child_frame_id;
static bool last_odometry_msg_is_valid = false;

void odometryCallback(const nav_msgs::Odometry& msg)
{
  std::memcpy(&last_odometry_msg, &msg, sizeof(nav_msgs::Odometry));
  
  last_odometry_msg_frame_id = msg.header.frame_id;
  last_odometry_msg_child_frame_id = msg.child_frame_id;
  
  last_odometry_msg_is_valid = true;
}

TEST(Odometry, test_odometry_straight)
{
  ros::NodeHandle node_handle("~");
  const std::string& odometry_topic("odometry");
  const std::string& left_twist_topic("left_twist");
  const std::string& right_twist_topic("right_twist");
  const std::string& frame_id("frame_id");
  const std::string& child_frame_id("child_frame_id");
  
  /* Setup the time scale
   * Motor: L  R     L  R     L  R     L  R
   *     v: 0  0     0  0     1  1     0  0
   * -------|--|-----|--|-----|--|-----|--|---...-> t
   *        t0 t1    t2 t3    t4 t5    t6 t7
   *
   * Between t2 and t4 the left motor traveled with an
   * effective velocity of 1 m/s. Likewise the right motor
   * did the same between t3 and t5. The odometry should be
   * correct after t7.
   */
  const ros::Duration dt = ros::Duration(1.0);
  const ros::Duration t_setup  = ros::Duration(0.1);
  /* The two nodes should fire within a millisecond of each other */
  const ros::Duration t_offset = ros::Duration(0.001);
  
  const ros::Time t0 = ros::Time::now();
  const ros::Time t1 = t0 + t_offset;
  const ros::Time t2 = t0 + dt;
  const ros::Time t3 = t1 + dt;
  const ros::Time t4 = t2 + dt;
  const ros::Time t5 = t3 + dt;
  const ros::Time t6 = t4 + dt;
  const ros::Time t7 = t5 + dt;
  
  geometry_msgs::TwistStamped left_motor_msg;
  geometry_msgs::TwistStamped right_motor_msg;
  
  /* Setup the motor publishers */
  ros::Publisher left_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(left_twist_topic, 1);
    
  ros::Publisher right_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(right_twist_topic, 1);
  
  /* Subscribe to the odometry */
  ros::Subscriber odometry_subscriber =
    node_handle.subscribe("odometry", 1, &odometryCallback);

  Odometry odometry(node_handle,
                    0.24, /* Test for wheel that are 24 cm appart */
                    left_twist_topic,
                    right_twist_topic,
                    odometry_topic,
                    frame_id,
                    child_frame_id);
                    
  /* 1. Publish the first set of motor messages.
   *    Simulate the right motor controller starting later
   *    than the left.
   */
  left_motor_msg.twist.linear.x  = 0.0;
  left_motor_msg.header.seq      = 1;
  left_motor_msg.header.stamp    = t0;
  right_motor_msg.twist.linear.x = 0.0;
  right_motor_msg.header.seq     = 1;
  right_motor_msg.header.stamp   = t1;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  ros::spinOnce(); /* Let the odometry receive both messages */

  /* 2. Publish the second set of motor messages.
   *    Verify the odometry is initialized correctly and not
   *    moving, given that the wheels are still after time
   *    dt.
   */
  left_motor_msg.twist.linear.x  = 0.0;
  left_motor_msg.header.seq      = 2;
  left_motor_msg.header.stamp    = t2;
  right_motor_msg.twist.linear.x = 0.0;
  right_motor_msg.header.seq     = 2;
  right_motor_msg.header.stamp   = t3;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  ros::spinOnce();
  
  odometry.update(); /* Let the odometry update itself */
  
  EXPECT_FALSE(last_odometry_msg_is_valid);
  
  ros::spinOnce();
  
  EXPECT_TRUE(last_odometry_msg_is_valid);
  
  EXPECT_EQ(last_odometry_msg_frame_id, frame_id);
  EXPECT_EQ(last_odometry_msg_child_frame_id, child_frame_id);
  
  EXPECT_EQ(last_odometry_msg.header.stamp, t2);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.z, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.w, 1.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.z, 0.0);
  
  last_odometry_msg_is_valid = false;
  
  /* 3. Publish the third set of motor messages.
   *    Verify that the odometry has adjusted its location
   *    estimate based on the messages it has received.
   */
  left_motor_msg.twist.linear.x  = 1.0;
  left_motor_msg.header.seq      = 3;
  left_motor_msg.header.stamp    = t4;
  right_motor_msg.twist.linear.x = 1.0;
  right_motor_msg.header.seq     = 3;
  right_motor_msg.header.stamp   = t5;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  
  left_motor_msg.twist.linear.x  = 0.0;
  left_motor_msg.header.seq      = 4;
  left_motor_msg.header.stamp    = t6;
  right_motor_msg.twist.linear.x = 0.0;
  right_motor_msg.header.seq     = 4;
  right_motor_msg.header.stamp   = t7;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  
  ros::spinOnce();
  
  odometry.update();
  
  ros::spinOnce();
  
  /* Expect the position to be less than a millimeter off */
  EXPECT_LT(1.0 - last_odometry_msg.pose.pose.position.x, 0.001);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.z, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.orientation.w, 1.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.z, 0.0);
  
  /* The time stamp should come from the last message */
  EXPECT_EQ(last_odometry_msg.header.stamp, t6);
  
  last_odometry_msg_is_valid = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ras_group8_odometry_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}