#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/pcl_mesh_traits.h"

namespace kimera_pgmo {

TEST(PclMeshTraits, stampTraitsCorrect) {
  static_assert(!traits::has_get_stamp<pcl::PointCloud<pcl::PointXYZ>>::value,
                "pcl::PointXYZ get failed");
  static_assert(!traits::has_get_stamp<pcl::PointCloud<pcl::PointXYZRGBA>>::value,
                "pcl::PointXYZRBA get failed");
  static_assert(traits::has_get_stamp<ConstStampedCloud<pcl::PointXYZ>>::value,
                "const stamped pcl::PointXYZ failed");
  static_assert(traits::has_get_stamp<ConstStampedCloud<pcl::PointXYZRGBA>>::value,
                "const stamped pcl::PointXYZRBA failed");
  SUCCEED();
}

}  // namespace kimera_pgmo
