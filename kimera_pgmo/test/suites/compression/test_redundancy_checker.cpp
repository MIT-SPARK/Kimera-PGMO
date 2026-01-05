#include <gtest/gtest.h>

#include "kimera_pgmo/compression/redundancy_checker.h"

namespace kimera_pgmo {

TEST(RedundancyChecker, SingleFaceCorrect) {
  RedundancyChecker checker;
  EXPECT_TRUE(checker.check({0, 1, 2}));
  checker.add({0, 1, 2});
  EXPECT_FALSE(checker.check({0, 1, 2}));
  EXPECT_FALSE(checker.check({1, 2, 0}));
  EXPECT_FALSE(checker.check({2, 0, 1}));

  EXPECT_TRUE(checker.check({0, 2, 1}));
  EXPECT_TRUE(checker.check({2, 1, 0}));
  EXPECT_TRUE(checker.check({1, 0, 2}));
}

TEST(RedundancyChecker, DuplicatesCorrect) {
  RedundancyChecker checker;
  checker.add({0, 1, 2});
  EXPECT_FALSE(checker.check({0, 1, 2}));
  checker.add({1, 2, 3});
}

TEST(RedundancyChecker, FlippedNormalCorrect) {
  RedundancyChecker checker;
  checker.add({0, 1, 3});
  checker.add({1, 4, 2});
  checker.add({3, 2, 5});
  EXPECT_TRUE(checker.check({1, 2, 3}));
  EXPECT_TRUE(checker.check({2, 3, 1}));
  EXPECT_TRUE(checker.check({3, 1, 2}));
  EXPECT_TRUE(checker.check({1, 3, 2}));
  EXPECT_TRUE(checker.check({3, 2, 1}));
  EXPECT_TRUE(checker.check({2, 1, 3}));
}

}  // namespace kimera_pgmo
