/**
 * @file   test_common_structs.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include "gtest/gtest.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "test_config.h"

namespace kimera_pgmo {
TEST(TestCommonStructs, stampFromSec) {
  double seconds = 105867.312345678;
  EXPECT_EQ(105867312345678, stampFromSec(seconds));
}

TEST(TestCommonStructs, stampToSec) {
  Timestamp stamp = 105867312345678;
  EXPECT_EQ(105867.312345678, stampToSec(stamp));
}

}  // namespace kimera_pgmo
