/**
 * @file   common_structs.cpp
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/utils/common_structs.h"

#include <chrono>

namespace kimera_pgmo {

Timestamp stampFromSec(double sec) {
  auto t = std::chrono::duration<double>(sec);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
}

double stampToSec(Timestamp stamp) {
  auto t = std::chrono::nanoseconds(stamp);
  return std::chrono::duration<double>(t).count();
}

}  // namespace kimera_pgmo
