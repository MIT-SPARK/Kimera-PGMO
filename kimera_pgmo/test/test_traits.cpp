#include <gtest/gtest.h>

#include <numeric>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/range_generator.h"

namespace kimera_pgmo {

TEST(TestTraits, stampTraitsCorrect) {
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

TEST(TestTraits, generator) {
  RangeGenerator generator(10);
  std::vector<int64_t> result(generator.begin(), generator.end());
  std::vector<int64_t> expected(10);
  std::iota(expected.begin(), expected.end(), 0);
  EXPECT_EQ(expected, result);

  std::vector<uint64_t> stamps{10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

  const RangeGenerator stamp_gen(stamps.size());
  auto bound = std::upper_bound(
      stamp_gen.begin(), stamp_gen.end(), 50, [&](auto value, auto idx) {
        return value < stamps.at(idx);
      });
  const auto diff = bound - generator.begin();
  EXPECT_EQ(diff, 5);
}

}  // namespace kimera_pgmo
