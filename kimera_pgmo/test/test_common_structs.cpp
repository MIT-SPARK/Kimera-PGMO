/**
 * @file   test_common_structs.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include "gtest/gtest.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "test_config.h"

namespace kimera_pgmo {
TEST(test_common_structs, stampFromSec) {
  double seconds = 105867.312345678;
  EXPECT_EQ(105867312345678, stampFromSec(seconds));
}

TEST(test_common_structs, stampToSec) {
  Timestamp stamp = 105867312345678;
  EXPECT_EQ(105867.312345678, stampToSec(stamp));
}

}  // namespace kimera_pgmo
