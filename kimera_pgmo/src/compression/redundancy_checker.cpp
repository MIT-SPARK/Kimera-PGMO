#include "kimera_pgmo/compression/redundancy_checker.h"

namespace kimera_pgmo {
namespace {

traits::Face reindexFace(const traits::Face& face) {
  if (face[0] < face[1] && face[0] < face[2]) {
    // 0 is smallest
    return {face[0], face[1], face[2]};
  }

  if (face[1] < face[2]) {
    // 1 is smallest (1 < 2 && 0 > 1 || 0 > 2)
    // rotate face to start with 1
    return {face[1], face[2], face[0]};
  }

  // 2 is smallest (1 > 2 && 0 > 1 || 0 > 2)
  // rotate face to start with 2
  return {face[2], face[0], face[1]};
}

}  // namespace

size_t RedundancyChecker::FaceHash::operator()(const traits::Face& face) const {
  size_t value = std::hash<size_t>{}(face[0]);
  // see
  // https://www.boost.org/doc/libs/latest/libs/container_hash/doc/html/hash.html#notes_hash_combine
  value ^= std::hash<size_t>{}(face[1]) + 0x9e3779b9 + (value << 6) + (value >> 2);
  value ^= std::hash<size_t>{}(face[2]) + 0x9e3779b9 + (value << 6) + (value >> 2);
  return value;
}

bool RedundancyChecker::check(const traits::Face& face) const {
  return !seen_.count(reindexFace(face));
}

void RedundancyChecker::add(const traits::Face& face) {
  seen_.insert(reindexFace(face));
}

void RedundancyChecker::clear() { seen_.clear(); }

}  // namespace kimera_pgmo
