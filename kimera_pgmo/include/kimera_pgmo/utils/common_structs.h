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

}  // namespace kimera_pgmo
