#include <gtest/gtest.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>

#include <stdexcept>

namespace kimera_pgmo {
namespace {

TEST(MeshDeltaConversion, RejectMalformedArrays) {
  kimera_pgmo_msgs::msg::MeshDelta msg;
  msg.previous_indices.push_back(0);
  EXPECT_THROW(conversions::from_ros(msg), std::invalid_argument);

  msg.previous_indices.clear();
  msg.num_archived_vertices = 1;
  EXPECT_THROW(conversions::from_ros(msg), std::invalid_argument);
}

}  // namespace
}  // namespace kimera_pgmo
