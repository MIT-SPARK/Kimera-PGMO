/**
 * @file   common_structs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once
#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

using traits::Timestamp;
Timestamp stampFromSec(double sec);
double stampToSec(Timestamp stamp);

struct StampedPoint {
  Timestamp timestamp_ns;
  Eigen::Vector3f pos;
};

}  // namespace kimera_pgmo
