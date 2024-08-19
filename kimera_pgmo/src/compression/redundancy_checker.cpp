/**
 * @file   reundancy_checker.h
 * @brief  Struct to check if faces are valid and have no duplicates
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include "kimera_pgmo/compression/redundancy_checker.h"

namespace kimera_pgmo {

bool RedunancyChecker::check(const Face& face) const {
  const bool present = hasEdge(face.v1, face.v2) && hasEdge(face.v2, face.v3) &&
                       hasEdge(face.v2, face.v3);
  return !present;
}

void RedunancyChecker::add(const Face& face) {
  addEdge(face.v1, face.v2);
  addEdge(face.v2, face.v3);
  addEdge(face.v3, face.v1);
}

bool RedunancyChecker::hasEdge(size_t source, size_t target) const {
  const auto iter = A.find(source);
  if (iter == A.end()) {
    return false;
  }

  return iter->second.count(target);
}

void RedunancyChecker::addEdge(size_t source, size_t target) {
  auto iter = A.find(source);
  if (iter == A.end()) {
    iter = A.insert({source, {}}).first;
  }

  iter->second.insert(target);
}

}  // namespace kimera_pgmo
