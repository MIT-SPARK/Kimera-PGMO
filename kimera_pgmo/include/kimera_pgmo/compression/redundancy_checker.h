#pragma once
#include <unordered_set>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

struct RedundancyChecker {
  struct FaceHash {
    size_t operator()(const traits::Face& face) const;
  };

  bool check(const traits::Face& face) const;
  void add(const traits::Face& face);
  void clear();

 private:
  std::unordered_set<traits::Face, FaceHash> seen_;
};

}  // namespace kimera_pgmo
