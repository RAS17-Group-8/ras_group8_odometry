#include <gtest/gtest.h>
//#include <ros/ros.h>
#include <ras_group8_odometry/TwistMsgBuffer.hpp>
#include <geometry_msgs/TwistStamped.h>

using namespace ras_group8_odometry;

TEST(TwistMsgBuffer, test_read_write)
{
  TwistMsgBuffer buffer;
  geometry_msgs::TwistStamped msg_a;
  geometry_msgs::TwistStamped msg_b;
  
  msg_a.header.seq = 0;
  msg_a.twist.linear.x = 42;
  msg_b.header.seq = 1;
  
  EXPECT_FALSE(buffer.isValid());
  
  buffer.insert(msg_a);
  
  EXPECT_FALSE(buffer.isValid());
  
  buffer.insert(msg_b);
  
  EXPECT_TRUE(buffer.isValid());
  EXPECT_EQ(buffer.tail()->twist.linear.x, 42);
}

TEST(TwistMsgBuffer, test_invalid_seq)
{
  TwistMsgBuffer buffer;
  geometry_msgs::TwistStamped msg_a;
  geometry_msgs::TwistStamped msg_b;
  
  msg_a.header.seq = 0;
  msg_b.header.seq = 2;
  
  buffer.insert(msg_a);
  buffer.insert(msg_b);
  
  EXPECT_TRUE(buffer.isInitialized());
  EXPECT_FALSE(buffer.isValid());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}