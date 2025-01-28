#include "kimera_pgmo_rviz/tf_event_buffer.h"

#include <ros/assert.h>
#include <rviz/frame_manager.h>

#include <functional>
#include <memory>
#include <unordered_map>

namespace kimera_pgmo {

TfEventBuffer::TfEventBuffer(rviz::FrameManager* frame_manager, const Config& config)
    : config(config), frame_manager_(frame_manager) {
  ROS_ASSERT_MSG(frame_manager_ != nullptr, "invalid frame manager!");
  last_update_time_ = std::chrono::high_resolution_clock::now();
}

void TfEventBuffer::addTransformQuery(const std::string& ns,
                                      const std::string& target_frame) {
  queries_[ns].target_frame = target_frame;
}

void TfEventBuffer::removeTransformQuery(const std::string& ns) {
  // TODO(lschmid): Should this trigger a final update? Probably fine to just clear.
  queries_.erase(ns);
}

std::vector<TfEventBuffer::Update> TfEventBuffer::getTransformUpdates() {
  // TODO(lschmid): This simply queries all transforms. Can be optimized in the
  // future.
  const auto now = std::chrono::high_resolution_clock::now();
  if (std::chrono::duration<float>(now - last_update_time_).count() <
      config.minimum_update_period) {
    return {};
  }
  last_update_time_ = now;

  std::vector<Update> updates;
  for (auto& [ns, query] : queries_) {
    auto update = getTransform(ns, true);
    if (!update) {
      continue;
    }

    if (!isNew(query, update->position, update->orientation)) {
      continue;
    }
    updates.push_back(*update);
  }
  return updates;
}

std::optional<TfEventBuffer::Update> TfEventBuffer::getTransform(const std::string& ns,
                                                                 bool update) {
  const auto it = queries_.find(ns);
  if (it == queries_.end()) {
    return std::nullopt;
  }
  auto& query = it->second;
  Update result;
  if (!update) {
    if (!query.valid) {
      return std::nullopt;
    }
    result.ns = ns;
    result.position = query.position;
    result.orientation = query.orientation;
    return result;
  }
  if (!frame_manager_->getTransform(
          query.target_frame, ros::Time(), result.position, result.orientation)) {
    return std::nullopt;
  }
  query.position = result.position;
  query.orientation = result.orientation;
  query.valid = true;
  return result;
}

bool TfEventBuffer::isNew(const Query& query,
                          const Ogre::Vector3& position,
                          const Ogre::Quaternion& orientation) const {
  // Check if the new transform is different from the old one.
  return !query.valid ||
         (position - query.position).length() > config.update_tolerance ||
         std::abs(orientation[0] - query.orientation[0]) > config.update_tolerance ||
         std::abs(orientation[1] - query.orientation[1]) > config.update_tolerance ||
         std::abs(orientation[2] - query.orientation[2]) > config.update_tolerance ||
         std::abs(orientation[3] - query.orientation[3]) > config.update_tolerance;
}

void TfEventBuffer::reset() { queries_.clear(); }

}  // namespace kimera_pgmo