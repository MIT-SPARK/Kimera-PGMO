/**
 * @file   pgmo_unit_tests.cpp
 * @brief  main file for unit tests
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include <gtest/gtest.h>

#include "kimera_pgmo/utils/logging.h"

int main(int argc, char **argv) {
  logging::Logger::addSink("cout", std::make_shared<logging::CoutSink>());
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
