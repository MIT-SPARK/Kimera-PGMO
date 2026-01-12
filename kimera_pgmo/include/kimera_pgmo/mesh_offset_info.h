#pragma once
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace kimera_pgmo {

//! Pointers to portions of the mesh that are archived and active
struct MeshOffsetInfo {
  using Remap = std::map<size_t, size_t>;

  //! Construct the mesh offset info
  MeshOffsetInfo(size_t archived_vertices = 0,
                 size_t prev_archived_vertices = 0,
                 size_t archived_faces = 0,
                 const std::shared_ptr<const Remap>& prev_to_curr = nullptr);

  //! Total number of archived vertices in the mesh
  size_t archived_vertices = 0;
  //! Previous number of archived vertices in the mesh
  size_t prev_archived_vertices = 0;
  //! Total number of archived faces in the mesh
  size_t archived_faces = 0;

  //! Convert local (delta) index to global (mesh) index
  size_t toGlobalVertex(size_t local_idx) const;

  //! Convert global (mesh) index to local (delta) index
  size_t toLocalVertex(size_t global_idx) const;

  /**
   * @brief Remap global vertex index from previous delta
   * @param[in] global_idx Global mesh vertex index
   * @return Remapped global index if vertex was not delted
   */
  std::optional<size_t> remapGlobalVertex(size_t global_idx) const;

  //! Statistics about remapped indices
  struct RemapStats {
    //! Minimum global index after remap
    size_t min_index = std::numeric_limits<size_t>::max();
    //! Maximum global index after remap
    size_t max_index = 0;
    //! Whether or not all indices are archived
    bool all_archived = false;
    //! Which indices (in the remapped container) were deleted
    std::set<size_t> deleted_indices;

    void addIndex(size_t idx);
  };

  /**
   * @brief Remap vertex indices from a previous mesh (in-place)
   * @param indices Indices to remap
   * @param info Optional tracking info for remapping
   */
  void remapVertexIndices(std::list<size_t>& indices, RemapStats* info = nullptr) const;

  /**
   * @brief Remap vertex indices from a previous mesh
   * @param indices Indices to remap
   * @param info Optional tracking info for remapping
   * @return Newly remapped indices
   */
  std::vector<size_t> remapVertexIndices(const std::vector<size_t>& indices,
                                         RemapStats* info = nullptr) const;

 private:
  std::shared_ptr<const Remap> prev_to_curr_;
};

}  // namespace kimera_pgmo
