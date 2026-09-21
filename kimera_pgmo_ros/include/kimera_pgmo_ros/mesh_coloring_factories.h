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

//! Factories validate settings and return independent processors with empty caches.
std::unique_ptr<MeshColoring> makeRgbColoring(const RgbColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeUniformColoring(
    const UniformColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeSemanticColoring(
    const SemanticColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeFirstSeenColoring(
    const FirstSeenColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeLastSeenColoring(
    const LastSeenColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeSeenDurationColoring(
    const SeenDurationColoringConfig& config = {});
std::unique_ptr<MeshColoring> makeSplitColoring(const SplitColoringConfig& config,
                                                std::shared_ptr<MeshColoring> child);

}  // namespace kimera_pgmo
