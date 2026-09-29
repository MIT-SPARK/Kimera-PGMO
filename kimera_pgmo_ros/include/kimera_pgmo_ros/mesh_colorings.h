#pragma once
#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "kimera_pgmo_ros/mesh_coloring.h"

namespace kimera_pgmo {

using MeshPalette = std::map<traits::Label, traits::Color>;
//! Inclusive bounds in nanoseconds, also used for observed durations.
using MeshTimeBounds = std::pair<traits::Timestamp, traits::Timestamp>;

struct RgbColoringConfig {
  traits::Color default_color{102, 102, 102, 255};
};

struct UniformColoringConfig {
  traits::Color color{102, 102, 102, 255};
};

struct SemanticColoringConfig {
  traits::Color default_color{102, 102, 102, 255};
  double alpha = 1.0;
  //! Missing selects the built-in 150-color palette; an empty map has no labels.
  std::optional<MeshPalette> palette;
};

struct FirstSeenColoringConfig {
  traits::Color invalid_color{0, 255, 0, 255};
  //! Missing selects automatic per-mesh bounds.
  std::optional<MeshTimeBounds> bounds;
};

struct LastSeenColoringConfig {
  traits::Color invalid_color{0, 255, 0, 255};
  std::optional<MeshTimeBounds> bounds;
};

struct SeenDurationColoringConfig {
  traits::Color invalid_color{0, 255, 0, 255};
  std::optional<MeshTimeBounds> bounds;
};

struct SplitColoringConfig {
  Eigen::Vector3f normal = Eigen::Vector3f::Ones();
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  traits::Color default_color{102, 102, 102, 255};
};

//! Built-in processors validate configuration at construction and own their caches.
class RgbMeshColoring : public MeshColoring {
 public:
  explicit RgbMeshColoring(const RgbColoringConfig& config = {});

  bool prepare(const MeshColoringView& /* mesh */, size_t /* first_changed */) override;

  traits::Color color(const MeshColoringView& mesh, size_t i) const override;

 private:
  traits::Color fallback_{102, 102, 102, 255};
};

class UniformMeshColoring : public MeshColoring {
 public:
  explicit UniformMeshColoring(const UniformColoringConfig& config = {});

  bool prepare(const MeshColoringView& /* mesh */, size_t /* first_changed */) override;

  traits::Color color(const MeshColoringView& /* mesh */,
                      size_t /* index */) const override;

 private:
  traits::Color color_{102, 102, 102, 255};
};

class SemanticMeshColoring : public MeshColoring {
 public:
  explicit SemanticMeshColoring(const SemanticColoringConfig& config = {});

  bool prepare(const MeshColoringView& /* mesh */, size_t /* first_changed */) override;

  traits::Color color(const MeshColoringView& mesh, size_t i) const override;

 private:
  traits::Color fallback_{102, 102, 102, 255};
  double alpha_ = 1.0;
  std::map<traits::Label, traits::Color> palette_;
};

class TimeMeshColoring : public MeshColoring {
 public:
  bool prepare(const MeshColoringView& mesh, size_t first_changed) override;

  traits::Color color(const MeshColoringView& mesh, size_t i) const override;

 protected:
  TimeMeshColoring(const traits::Color& invalid,
                   const std::optional<MeshTimeBounds>& bounds);
  virtual std::optional<uint64_t> value(const traits::VertexTraits& traits) const = 0;
  virtual bool zeroMinimum() const { return false; }

 private:
  traits::Color invalid_{0, 255, 0, 255};
  std::optional<std::pair<uint64_t, uint64_t>> bounds_;
  std::pair<uint64_t, uint64_t> automatic_bounds_{0, 0};
  std::vector<std::optional<uint64_t>> values_;
  std::map<uint64_t, size_t> counts_;
};

class FirstSeenMeshColoring : public TimeMeshColoring {
 public:
  explicit FirstSeenMeshColoring(const FirstSeenColoringConfig& config = {});

 protected:
  std::optional<uint64_t> value(const traits::VertexTraits& t) const override;
};

class LastSeenMeshColoring : public TimeMeshColoring {
 public:
  explicit LastSeenMeshColoring(const LastSeenColoringConfig& config = {});

 protected:
  std::optional<uint64_t> value(const traits::VertexTraits& t) const override;
};

class SeenDurationMeshColoring : public TimeMeshColoring {
 public:
  explicit SeenDurationMeshColoring(const SeenDurationColoringConfig& config = {});

 protected:
  bool zeroMinimum() const override;

  std::optional<uint64_t> value(const traits::VertexTraits& t) const override;
};

class SplitMeshColoring : public MeshColoring {
 public:
  SplitMeshColoring(const SplitColoringConfig& config,
                    std::shared_ptr<MeshColoring> child);

  bool prepare(const MeshColoringView& mesh, size_t first_changed) override;

  traits::Color color(const MeshColoringView& mesh, size_t i) const override;

 private:
  Eigen::Vector3f normal_ = Eigen::Vector3f::Ones();
  Eigen::Vector3f origin_ = Eigen::Vector3f::Zero();
  traits::Color fallback_{102, 102, 102, 255};
  std::shared_ptr<MeshColoring> coloring_;
};

}  // namespace kimera_pgmo
