/**
 * @file   reundancy_checker.h
 * @brief  Struct to check if faces are valid and have no duplicates
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include <map>
#include <set>

#include "kimera_pgmo/mesh_delta.h"

namespace kimera_pgmo {

struct RedunancyChecker {
  using SparseAdjacencyMatrix = std::map<size_t, std::set<size_t>>;

  bool check(const Face& face) const;
  void add(const Face& face);
  bool hasEdge(size_t source, size_t target) const;
  void addEdge(size_t source, size_t target);

  SparseAdjacencyMatrix A;
};

}  // namespace kimera_pgmo
