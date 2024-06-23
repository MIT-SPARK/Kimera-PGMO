/**
 * @file   common_functions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kimera_pgmo/mesh_types.h>
#include <pose_graph_tools_msgs/PoseGraph.h>

namespace kimera_pgmo::conversions {

using GraphMsgPtr = pose_graph_tools_msgs::PoseGraph::ConstPtr;
using kimera_pgmo::traits::Timestamp;

/*! \brief Converts a ros pose type to gtsam Pose3
 *  - transform: ros geometry_msgs pose type
 *  - outputs pose as gtsam Pose3
 */
gtsam::Pose3 RosToGtsam(const geometry_msgs::Pose& transform);

/*! \brief Converts a gtsam pose type to ros geometry message
 *  - pose: gtsam pose3
 *  - outputs pose as geometry_msg/pose
 */
geometry_msgs::Pose GtsamToRos(const gtsam::Pose3& pose);

/*! \brief Convert gtsam factor graph and estimated values to ros graph msg for
 * publication
 *  - factors: gtsam NonlinearFactorGraph storing all added factors
 *  - values: gtsam Values storing estimated values
 *  - timestamps: map from robot id to timestamps of the poses in the
 * trajectories
 *  - gnc_weights: weights obtained from gnc optimizer for inlier-outlier
 * sorting
 */
GraphMsgPtr GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                            const gtsam::Values& values,
                            const std::map<size_t, std::vector<Timestamp>>& timestamps,
                            const gtsam::Vector& gnc_weights = gtsam::Vector(),
                            const std::string& frame_id = "world");

}  // namespace kimera_pgmo::conversions
