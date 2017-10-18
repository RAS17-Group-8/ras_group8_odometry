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
  
  const ros::Time now = ros::Time::now();
  const ros::Duration dt = ros::Duration(1.0);
  const ros::Duration t_offset = ros::Duration(0.1);
  
  geometry_msgs::TwistStamped left_motor_msg;
  geometry_msgs::TwistStamped right_motor_msg;
  
  ros::Publisher left_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(left_twist_topic, 1);
    
  ros::Publisher right_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(right_twist_topic, 1);
    
  ros::Subscriber odometry_subscriber =
    node_handle.subscribe("odometry", 1, &odometryCallback);

  Odometry odometry(node_handle,
                    1.0,
                    left_twist_topic,
                    right_twist_topic,
                    odometry_topic,
                    frame_id,
                    child_frame_id);

  /* Verify the odometry is initialized correctly and not
   * moving, given that the wheels are still.
   */
  left_motor_msg.twist.linear.x = 0.0;
  left_motor_msg.header.stamp = now;
  right_motor_msg.twist.linear.x = 0.0;
  right_motor_msg.header.stamp = now + t_offset;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  ros::spinOnce();
  odometry.update();
  
  EXPECT_FALSE(last_odometry_msg_is_valid);
  
  ros::spinOnce();
  
  EXPECT_TRUE(last_odometry_msg_is_valid);
  
  EXPECT_EQ(last_odometry_msg_frame_id, frame_id);
  EXPECT_EQ(last_odometry_msg_child_frame_id, child_frame_id);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.z, 0.0);
  
  /* TODO: test last_odometry_msg.pose.pose.orientation */
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.z, 0.0);
  
  last_odometry_msg_is_valid = false;
  
  /* Test the odometry when both wheels are moving.
   */
  left_motor_msg.twist.linear.x = 1.0;
  left_motor_msg.header.stamp = now + dt;
  right_motor_msg.twist.linear.x = 1.0;
  right_motor_msg.header.stamp = now + t_offset + dt;
  
  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  ros::spinOnce();
  odometry.update();
  
  ros::spinOnce();
  
  EXPECT_TRUE(last_odometry_msg_is_valid);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.x, 1.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.x, 1.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.z, 0.0);
  
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.z, 0.0);
  
  /* The time stamp should come from the last message */
  EXPECT_FLOAT_EQ(last_odometry_msg.header.stamp.toSec(), (now + t_offset + dt).toSec());
  
  last_odometry_msg_is_valid = false;
}

TEST(Odometry, test_odometry_turn)
{
  ros::NodeHandle node_handle("~");
  const std::string& odometry_topic("odometry");
  const std::string& left_twist_topic("left_twist");
  const std::string& right_twist_topic("right_twist");
  const std::string& frame_id("frame_id");
  const std::string& child_frame_id("child_frame_id");

  const ros::Time now = ros::Time::now();
  const ros::Duration dt = ros::Duration(1.0);
  const ros::Duration t_offset = ros::Duration(0.1);

  geometry_msgs::TwistStamped left_motor_msg;
  geometry_msgs::TwistStamped right_motor_msg;

  ros::Publisher left_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(left_twist_topic, 1);

  ros::Publisher right_motor =
    node_handle.advertise<geometry_msgs::TwistStamped>(right_twist_topic, 1);

  ros::Subscriber odometry_subscriber =
    node_handle.subscribe("odometry", 1, &odometryCallback);

  Odometry odometry(node_handle,
                    1.0,
                    left_twist_topic,
                    right_twist_topic,
                    odometry_topic,
                    frame_id,
                    child_frame_id);
  
  /* Test the odometry when both wheels are moving.
   */
  left_motor_msg.twist.linear.x = -1.0;
  left_motor_msg.header.stamp = now + dt;
  right_motor_msg.twist.linear.x = 1.0;
  right_motor_msg.header.stamp = now + t_offset + dt;

  left_motor.publish(left_motor_msg);
  right_motor.publish(right_motor_msg);
  ros::spinOnce();
  odometry.update();
  
  EXPECT_FALSE(last_odometry_msg_is_valid);
  
  ros::spinOnce();

  EXPECT_TRUE(last_odometry_msg_is_valid);

  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.pose.pose.position.z, 0.0);

  /* TODO: test last_odometry_msg.pose.pose.orientation */

  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.linear.z, 0.0);

  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.x, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.y, 0.0);
  EXPECT_FLOAT_EQ(last_odometry_msg.twist.twist.angular.z, atan(2.0));

  /* The time stamp should come from the last message */
  EXPECT_FLOAT_EQ(last_odometry_msg.header.stamp.toSec(), (now + t_offset + dt).toSec());

  last_odometry_msg_is_valid = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ras_group8_odometry_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}