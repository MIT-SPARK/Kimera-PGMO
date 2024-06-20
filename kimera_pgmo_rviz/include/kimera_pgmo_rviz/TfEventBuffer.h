#pragma once

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <chrono>
#include <memory>
#include <optional>
#include <unordered_map>

namespace rviz {
class FrameManager;
}

namespace kimera_pgmo {

class MeshVisual;

/**
 * @brief Class to listen for transform updates for specific frames to trigger transform
 * updates.
 * TODO(lschmid): Currently simply looks for updated transforms. Can implement a fancier
 * buffer in the future if need be.
 */
class TfEventBuffer {
 public:
  struct Config {
    float minimum_update_period = 0.05f;  // minimum update duration in seconds.
    float update_tolerance =
        1e-3f;  // update tolerance for when a transform is considered new.
  } config;

  TfEventBuffer(rviz::FrameManager* frame_manager, const Config& config);
  virtual ~TfEventBuffer() = default;

  /**
   * @brief Struct to get the results (updates) of transform queries.
   */
  struct Update {
    std::string ns;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };

  /**
   * @brief Add a transform query to the buffer to lookup. If the query exists already
   * its target frame will be updated.
   * @param ns The namespace of the visual requesting the transform.
   * @param target_frame The target frame to lookup.
   */
  void addTransformQuery(const std::string& ns, const std::string& target_frame);

  /**
   * @brief Remove a transform query to the buffer to lookup.
   */
  void removeTransformQuery(const std::string& ns);

  /**
   * @brief Get all updated transforms.
   */
  std::vector<Update> getTransformUpdates();

  /**
   * @brief Query the transform for a specific namespace.
   * @param ns The namespace of the visual requesting the transform.
   * @param update If true, the get the most recent transform and update the buffer.
   * Otherwise get the last known transform.
   */
  std::optional<Update> getTransform(const std::string& ns, bool update = true);

  /**
   * @brief Clear all queries.
   */
  void reset();

 private:
  rviz::FrameManager* frame_manager_;

  struct Query {
    std::string target_frame;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    bool valid = false;
  };

  std::unordered_map<std::string, Query> queries_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_update_time_;

  bool isNew(const Query& query,
             const Ogre::Vector3& position,
             const Ogre::Quaternion& orientation) const;
};

}  // namespace kimera_pgmo
