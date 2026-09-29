#pragma once
#include <kimera_pgmo/mesh_types.h>

#include <string>
#include <vector>

namespace kimera_pgmo {

//! Non-owning view, valid only during a coloring call. Indices are mesh indices,
//! not chunk-local indices. Neither the view nor its references may be retained.
struct MeshColoringView {
  const std::vector<traits::Vertex>& vertices;
  const std::vector<traits::Face>& faces;
  const std::string& ns;
};

//! Ogre-, Qt-, and RViz-independent coloring processor.
//! Each mesh owns a separate instance. Calls are serialized by the display.
//! RViz property editors and plugin registration live in kimera_pgmo_rviz.
class MeshColoring {
 public:
  virtual ~MeshColoring() = default;

  //! Called after a mesh update. Vertices before first_changed_vertex are unchanged;
  //! the remaining suffix may have been replaced, resized, or remapped. Zero means
  //! a full replacement. Return true if colors in the unchanged prefix also changed.
  //! The conservative default lets downstream plugins use mesh-wide statistics.
  virtual bool prepare(const MeshColoringView& /* mesh */,
                       size_t /* first_changed_vertex */) {
    return true;
  }

  //! Return straight RGBA bytes. Source vertex colors must never be overwritten.
  virtual traits::Color color(const MeshColoringView& mesh, size_t index) const = 0;
};

}  // namespace kimera_pgmo
