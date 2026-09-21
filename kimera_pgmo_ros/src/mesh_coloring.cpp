#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>

#include "kimera_pgmo_ros/mesh_coloring_factories.h"

namespace kimera_pgmo {
namespace {

static const std::vector<std::array<uint8_t, 3>> custom_150_palette{
    {0, 102, 51},    {0, 255, 0},     {4, 74, 246},    {191, 191, 181}, {72, 90, 13},
    {204, 0, 102},   {196, 26, 252},  {254, 217, 251}, {127, 255, 255}, {133, 87, 54},
    {130, 26, 50},   {10, 5, 65},     {3, 75, 135},    {18, 124, 89},   {21, 182, 126},
    {90, 171, 130},  {189, 217, 119}, {188, 63, 11},   {86, 82, 249},   {196, 221, 253},
    {187, 78, 62},   {188, 49, 167},  {182, 161, 248}, {112, 201, 196}, {11, 41, 99},
    {211, 102, 1},   {4, 227, 101},   {147, 201, 250}, {255, 0, 0},     {224, 33, 3},
    {254, 74, 17},   {255, 127, 0},   {176, 188, 66},  {247, 29, 54},   {74, 117, 80},
    {71, 56, 199},   {250, 107, 216}, {4, 204, 229},   {25, 208, 53},   {52, 249, 65},
    {70, 220, 121},  {68, 86, 156},   {7, 150, 36},    {255, 127, 127}, {64, 46, 6},
    {139, 167, 123}, {18, 165, 78},   {157, 166, 28},  {9, 163, 248},   {191, 67, 218},
    {135, 243, 61},  {81, 119, 190},  {0, 126, 204},   {228, 73, 252},  {227, 108, 163},
    {127, 255, 127}, {50, 211, 203},  {3, 190, 177},   {127, 255, 0},   {11, 59, 10},
    {108, 38, 143},  {243, 33, 254},  {182, 106, 251}, {120, 251, 191}, {145, 115, 208},
    {48, 129, 250},  {43, 78, 65},    {73, 0, 197},    {0, 249, 143},   {80, 160, 69},
    {200, 252, 62},  {127, 39, 198},  {139, 79, 186},  {155, 225, 169}, {239, 148, 63},
    {30, 183, 5},    {137, 90, 3},    {49, 35, 155},   {242, 186, 113}, {127, 0, 0},
    {191, 148, 1},   {134, 147, 65},  {175, 254, 104}, {84, 183, 23},   {12, 87, 183},
    {1, 20, 178},    {185, 224, 0},   {255, 255, 127}, {44, 252, 176},  {62, 186, 252},
    {3, 253, 46},    {41, 208, 156},  {138, 94, 126},  {32, 31, 231},   {192, 120, 193},
    {231, 66, 87},   {105, 0, 153},   {251, 216, 190}, {228, 45, 198},  {0, 0, 255},
    {67, 234, 247},  {156, 4, 152},   {253, 219, 57},  {183, 148, 126}, {246, 154, 179},
    {127, 127, 255}, {0, 255, 255},   {225, 189, 224}, {47, 253, 125},  {47, 0, 127},
    {92, 207, 58},   {173, 253, 235}, {251, 78, 170},  {164, 41, 91},   {0, 127, 0},
    {65, 43, 54},    {196, 100, 129}, {143, 157, 184}, {234, 190, 2},   {0, 234, 191},
    {191, 254, 179}, {149, 1, 215},   {67, 231, 11},   {131, 195, 0},   {188, 121, 62},
    {70, 143, 13},   {106, 168, 230}, {231, 106, 88},  {255, 255, 0},   {121, 54, 15},
    {127, 255, 0},   {248, 157, 252}, {63, 7, 14},     {146, 63, 249},  {185, 8, 32},
    {247, 0, 223},   {234, 252, 199}, {91, 29, 248},   {122, 0, 89},    {41, 99, 220},
    {28, 132, 140},  {127, 209, 111}, {110, 54, 79},   {63, 68, 107},   {214, 0, 171},
    {71, 9, 82},     {224, 187, 57},  {253, 20, 130},  {127, 127, 0},   {113, 132, 132},
};

traits::Color blend(const traits::Color& a, const traits::Color& b, double ratio) {
  traits::Color result;
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = static_cast<uint8_t>(std::lround((1.0 - ratio) * a[i] + ratio * b[i]));
  }

  return result;
}

traits::Color timeColor(double ratio) {
  static const std::array<traits::Color, 5> palette{{{0, 0, 0, 255},
                                                     {145, 20, 145, 255},
                                                     {255, 138, 0, 255},
                                                     {255, 230, 40, 255},
                                                     {255, 255, 255, 255}}};
  const auto scaled = std::clamp(ratio, 0.0, 1.0) * (palette.size() - 1);
  const auto lower = std::min(static_cast<size_t>(scaled), palette.size() - 2);
  return blend(palette[lower], palette[lower + 1], scaled - lower);
}

class RgbMeshColoring : public MeshColoring {
 public:
  explicit RgbMeshColoring(const RgbColoringConfig& config)
      : fallback_(config.default_color) {}

  bool prepare(const MeshColoringView&, size_t) override { return false; }

  traits::Color color(const MeshColoringView& mesh, size_t i) const override {
    const auto& traits = mesh.vertices[i].traits;
    return traits.properties.has_color ? traits.color : fallback_;
  }

 private:
  traits::Color fallback_{102, 102, 102, 255};
};

class UniformMeshColoring : public MeshColoring {
 public:
  explicit UniformMeshColoring(const UniformColoringConfig& config)
      : color_(config.color) {}

  bool prepare(const MeshColoringView&, size_t) override { return false; }

  traits::Color color(const MeshColoringView&, size_t) const override { return color_; }

 private:
  traits::Color color_{102, 102, 102, 255};
};

class SemanticMeshColoring : public MeshColoring {
 public:
  explicit SemanticMeshColoring(const SemanticColoringConfig& config)
      : fallback_(config.default_color), alpha_(config.alpha) {
    if (!std::isfinite(alpha_) || alpha_ < 0.0 || alpha_ > 1.0) {
      throw std::invalid_argument("Semantic blend must be in [0, 1]");
    }

    if (config.palette) {
      palette_ = *config.palette;
      return;
    }

    for (size_t i = 0; i < custom_150_palette.size(); ++i) {
      const auto& c = custom_150_palette[i];
      palette_[i] = {c[0], c[1], c[2], 255};
    }
  }

  bool prepare(const MeshColoringView&, size_t) override { return false; }

  traits::Color color(const MeshColoringView& mesh, size_t i) const override {
    const auto& traits = mesh.vertices[i].traits;
    const auto source = traits.properties.has_color ? traits.color : fallback_;
    if (!traits.properties.has_label) {
      return source;
    }

    const auto iter = palette_.find(traits.label);
    const auto label_color = iter == palette_.end() ? fallback_ : iter->second;
    return blend(source, label_color, alpha_);
  }

 private:
  traits::Color fallback_{102, 102, 102, 255};
  double alpha_ = 1.0;
  std::map<traits::Label, traits::Color> palette_;
};

class TimeMeshColoring : public MeshColoring {
 public:
  TimeMeshColoring(const traits::Color& invalid,
                   const std::optional<MeshTimeBounds>& bounds)
      : invalid_(invalid), bounds_(bounds) {
    if (bounds_ && bounds_->first > bounds_->second) {
      throw std::invalid_argument("Minimum time must not exceed maximum time");
    }
  }

  bool prepare(const MeshColoringView& mesh, size_t first_changed) override {
    if (bounds_) {
      return false;
    }

    // A multiset of timestamp counts supports removal/replacement of the active
    // suffix without scanning the archived mesh on every delta.
    first_changed = std::min(first_changed, values_.size());
    for (size_t i = first_changed; i < values_.size(); ++i) {
      if (!values_[i]) {
        continue;
      }

      const auto iter = counts_.find(*values_[i]);
      --iter->second;
      if (!iter->second) {
        counts_.erase(iter);
      }
    }

    values_.resize(mesh.vertices.size());
    for (size_t i = first_changed; i < values_.size(); ++i) {
      values_[i] = value(mesh.vertices[i].traits);
      if (values_[i]) {
        ++counts_[*values_[i]];
      }
    }

    const auto previous = automatic_bounds_;
    automatic_bounds_ = counts_.empty() ? std::make_pair(uint64_t{0}, uint64_t{0})
                                        : std::make_pair(counts_.begin()->first,
                                                         counts_.rbegin()->first);
    if (zeroMinimum()) {
      automatic_bounds_.first = 0;
    }

    return automatic_bounds_ != previous;
  }

  traits::Color color(const MeshColoringView& mesh, size_t i) const override {
    const auto stamp = value(mesh.vertices[i].traits);
    if (!stamp) {
      return invalid_;
    }

    const auto [min, max] = bounds_.value_or(automatic_bounds_);
    if (*stamp <= min || min == max) {
      return timeColor(0.0);
    }

    if (*stamp >= max) {
      return timeColor(1.0);
    }

    return timeColor(static_cast<double>(*stamp - min) /
                     static_cast<double>(max - min));
  }

 protected:
  virtual std::optional<uint64_t> value(const traits::VertexTraits&) const = 0;
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
  explicit FirstSeenMeshColoring(const FirstSeenColoringConfig& config)
      : TimeMeshColoring(config.invalid_color, config.bounds) {}

 protected:
  std::optional<uint64_t> value(const traits::VertexTraits& t) const override {
    return t.properties.has_first_seen_stamp && t.first_seen_stamp
               ? std::optional<uint64_t>(t.first_seen_stamp)
               : std::nullopt;
  }
};

class LastSeenMeshColoring : public TimeMeshColoring {
 public:
  explicit LastSeenMeshColoring(const LastSeenColoringConfig& config)
      : TimeMeshColoring(config.invalid_color, config.bounds) {}

 protected:
  std::optional<uint64_t> value(const traits::VertexTraits& t) const override {
    return t.properties.has_stamp && t.stamp ? std::optional<uint64_t>(t.stamp)
                                             : std::nullopt;
  }
};

class SeenDurationMeshColoring : public TimeMeshColoring {
 public:
  explicit SeenDurationMeshColoring(const SeenDurationColoringConfig& config)
      : TimeMeshColoring(config.invalid_color, config.bounds) {}

 protected:
  bool zeroMinimum() const override { return true; }

  std::optional<uint64_t> value(const traits::VertexTraits& t) const override {
    if (!t.properties.has_stamp || !t.properties.has_first_seen_stamp || !t.stamp ||
        !t.first_seen_stamp || t.stamp < t.first_seen_stamp) {
      return std::nullopt;
    }

    return t.stamp - t.first_seen_stamp;
  }
};

class SplitMeshColoring : public MeshColoring {
 public:
  SplitMeshColoring(const SplitColoringConfig& config,
                    std::shared_ptr<MeshColoring> child)
      : normal_(config.normal),
        origin_(config.origin),
        fallback_(config.default_color),
        coloring_(std::move(child)) {
    if (!normal_.allFinite() || normal_.squaredNorm() == 0 || !origin_.allFinite()) {
      throw std::invalid_argument(
          "Split plane must have a finite nonzero normal and finite origin");
    }

    if (!coloring_) {
      throw std::invalid_argument("Split coloring requires a child processor");
    }
  }

  bool prepare(const MeshColoringView& mesh, size_t first_changed) override {
    return coloring_->prepare(mesh, first_changed);
  }

  traits::Color color(const MeshColoringView& mesh, size_t i) const override {
    const auto& vertex = mesh.vertices[i];
    if (normal_.dot(vertex.pos - origin_) >= 0.0f) {
      return coloring_->color(mesh, i);
    }

    return vertex.traits.properties.has_color ? vertex.traits.color : fallback_;
  }

 private:
  Eigen::Vector3f normal_ = Eigen::Vector3f::Ones();
  Eigen::Vector3f origin_ = Eigen::Vector3f::Zero();
  traits::Color fallback_{102, 102, 102, 255};
  std::shared_ptr<MeshColoring> coloring_;
};

}  // namespace

std::unique_ptr<MeshColoring> makeRgbColoring(const RgbColoringConfig& config) {
  return std::make_unique<RgbMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeUniformColoring(const UniformColoringConfig& config) {
  return std::make_unique<UniformMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeSemanticColoring(
    const SemanticColoringConfig& config) {
  return std::make_unique<SemanticMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeFirstSeenColoring(
    const FirstSeenColoringConfig& config) {
  return std::make_unique<FirstSeenMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeLastSeenColoring(
    const LastSeenColoringConfig& config) {
  return std::make_unique<LastSeenMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeSeenDurationColoring(
    const SeenDurationColoringConfig& config) {
  return std::make_unique<SeenDurationMeshColoring>(config);
}

std::unique_ptr<MeshColoring> makeSplitColoring(const SplitColoringConfig& config,
                                                std::shared_ptr<MeshColoring> child) {
  return std::make_unique<SplitMeshColoring>(config, std::move(child));
}

}  // namespace kimera_pgmo
