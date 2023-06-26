/**
 * @file   pgmo_unit_tests.cpp
 * @brief  main file for unit tests
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include <gtest/gtest.h>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
