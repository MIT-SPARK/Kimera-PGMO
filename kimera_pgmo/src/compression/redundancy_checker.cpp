#include "kimera_pgmo/compression/redundancy_checker.h"

namespace kimera_pgmo {

bool RedundancyChecker::check(const traits::Face& face) const {
  // TODO(nathan) this is bad and needs to be smarter
  const bool present = hasEdge(face[0], face[1]) && hasEdge(face[1], face[2]) &&
                       hasEdge(face[2], face[0]);
  return !present;
}

void RedundancyChecker::add(const traits::Face& face) {
  addEdge(face[0], face[1]);
  addEdge(face[1], face[2]);
  addEdge(face[2], face[0]);
}

bool RedundancyChecker::hasEdge(size_t source, size_t target) const {
  const auto iter = A.find(source);
  if (iter == A.end()) {
    return false;
  }

  return iter->second.count(target);
}

void RedundancyChecker::addEdge(size_t source, size_t target) {
  auto iter = A.find(source);
  if (iter == A.end()) {
    iter = A.insert({source, {}}).first;
  }

  iter->second.insert(target);
}

}  // namespace kimera_pgmo
