/**
 * @file   test_kimera_pgmo.cpp
 * @brief  Unit-tests for main kimera pgmo class
 * @author Yun Chang
 */
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/KimeraPgmoMulti.h"

namespace kimera_pgmo {

class KimeraPgmoMultiTest : public ::testing::Test {
 protected:
  KimeraPgmoMultiTest() {
    system("rosparam set num_robots 3");
    system("rosparam set frame_id world");
    system("rosparam set compression_time_horizon 10.0");
    system("rosparam set run_mode 0");
    system("rosparam set use_msg_time true");
    system("rosparam set output_prefix test");
    system("rosparam set embed_trajectory_delta_t 3.0");
    system("rosparam set d_graph_resolution 0.1");
    system("rosparam set rpgo/translation_threshold 10.0");
    system("rosparam set rpgo/rotation_threshold 10.0");
  }
  ~KimeraPgmoMultiTest() {}

  inline size_t GetNumMeshPublishers() const {
    return pgmo_multi_.optimized_mesh_pub_.size();
  }
  inline size_t GetNumPoseGraphSubscribers() const {
    return pgmo_multi_.pose_graph_incremental_sub_.size();
  }
  inline size_t GetNumFullMeshSubscribers() const {
    return pgmo_multi_.full_mesh_sub_.size();
  }
  inline size_t GetNumIncMeshSubscribers() const {
    return pgmo_multi_.incremental_mesh_sub_.size();
  }
  inline size_t GetNumPathSubscribers() const {
    return pgmo_multi_.path_callback_sub_.size();
  }

  KimeraPgmoMulti pgmo_multi_;
};

TEST_F(KimeraPgmoMultiTest, initialize) {
  ros::NodeHandle nh;
  bool init = pgmo_multi_.initialize(nh);

  ASSERT_TRUE(init);

  EXPECT_EQ(3, GetNumMeshPublishers());
  EXPECT_EQ(3, GetNumPoseGraphSubscribers());
  EXPECT_EQ(3, GetNumFullMeshSubscribers());
  EXPECT_EQ(3, GetNumIncMeshSubscribers());
  EXPECT_EQ(3, GetNumPathSubscribers());
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo_multi");
  return RUN_ALL_TESTS();
}
