/**
 * @file   test_kimera_pgmo.cpp
 * @brief  Unit-tests for main kimera pgmo class
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

class KimeraPgmoTest : public ::testing::Test {
 protected:
  KimeraPgmoTest() {}

  ~KimeraPgmoTest() {}
};

TEST_F(KimeraPgmoTest, initialize) {}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}
