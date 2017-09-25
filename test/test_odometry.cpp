#include <gtest/gtest.h>
#include <ras_group8_odometry/Odometry.hpp>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}