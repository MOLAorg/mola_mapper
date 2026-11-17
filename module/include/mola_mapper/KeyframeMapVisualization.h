/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/
/**
 * @file   KeyframeMapVisualization.h
 * @brief  GUI map built from the keyframe clouds at their optimized poses.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

namespace mola::mapper
{
/// How the keyframe map is built and drawn.
struct KeyframeMapParams
{
  /// Voxel size [m] of each keyframe cloud.
  double voxel = 0.20;
  /// Points kept by their range [m] from the vehicle.
  double min_range = 1.5;
  double max_range = 50.0;
  /** A keyframe closer than this [m] to another one already in the map adds
   *  no cloud (keeps the point count bounded on revisits and slow motion). */
  double min_spacing = 2.5;
  /** Colormap limits [m] of the point height relative to the keyframe that
   *  observed it (fixed, so colors do not change during a run). */
  double color_min = -2.0;
  double color_max = 12.0;
  float point_size = 1.0f;
  /// Map corrections glide into place over this time [s] (0: they jump).
  double glide = 1.5;
};

/// Turbo colormap, x in [0,1] (polynomial approximation).
[[nodiscard]] mrpt::img::TColor turbo_color(double x);

/** The global map shown by the GUI: one colored cloud per contributing
 *  keyframe, built once from its observations (with an sm2mm pipeline: IMU
 *  de-skew from the keyframe velocity buffer, range filter, voxel decimation)
 *  in the keyframe frame, and drawn at the keyframe's current pose, so loop
 *  closures visibly correct the map. Points are colored by their height
 *  relative to the keyframe, in the map frame, which works on slopes and
 *  stacked floors alike.
 *
 *  Not thread-safe: use it from one thread. The scene nodes (one per cloud)
 *  are kept for the whole run, so the visualizer keeps their GPU buffers;
 *  once handed to the scene, they and their clouds must only be changed from
 *  the render thread (pose_changes(), point size).
 */
class KeyframeMapVisualization
{
public:
  explicit KeyframeMapVisualization(const KeyframeMapParams & p);

  /// A keyframe to consider, by index in the mapper keyframe list.
  struct KeyframeData
  {
    std::size_t index = 0;
    /// Shallow copy of the keyframe observations.
    mrpt::obs::CSensoryFrame observations;
  };

  /** Considers new keyframes, in increasing index order, starting at
   *  num_considered(), and builds the clouds of those that contribute.
   *  `poses_in_map` holds the current pose of every keyframe (at least up to
   *  the last one given). Stops early once `max_seconds` elapse: the rest
   *  are left for the next call (see num_considered()).
   *  \return The number of new clouds. */
  std::size_t add_keyframes(
    const std::vector<KeyframeData> & kfs, const std::vector<mrpt::poses::CPose3D> & poses_in_map,
    double max_seconds = 1e9);

  /// Keyframes already considered (indices 0 .. num_considered()-1).
  [[nodiscard]] std::size_t num_considered() const { return num_considered_; }
  /// Keyframes with a cloud in the map.
  [[nodiscard]] std::size_t num_clouds() const { return clouds_.size(); }
  /// Total points in the map.
  [[nodiscard]] std::size_t num_points() const { return num_points_; }

  /** The scene: a new container with the node of each contributing keyframe
   *  (a CSetOfObjects holding its cloud, at its pose). The same nodes every
   *  time: publish it again only when clouds were added. */
  [[nodiscard]] mrpt::viz::CSetOfObjects::Ptr scene() const;

  /// A node, its new pose and the one it was drawn at.
  struct NodePose
  {
    mrpt::viz::CSetOfObjects::Ptr node;
    mrpt::poses::CPose3D pose;
    mrpt::poses::CPose3D from;
  };
  /** The nodes whose keyframe moved since last drawn (e.g. after a loop
   *  closure), with their new poses in `poses_in_map`, to apply from the
   *  render thread. They are taken as drawn from now on. */
  [[nodiscard]] std::vector<NodePose> pose_changes(
    const std::vector<mrpt::poses::CPose3D> & poses_in_map);

  /// Point size of new clouds (existing ones are not changed).
  void set_point_size(float size) { params_.point_size = size; }
  [[nodiscard]] float point_size() const { return params_.point_size; }

  /// The clouds, e.g. to change their point size from the render thread.
  [[nodiscard]] std::vector<mrpt::viz::CPointCloudColoured::Ptr> clouds() const;

  /// The cloud of one keyframe, in its frame (nullptr if empty or invalid).
  [[nodiscard]] mrpt::viz::CPointCloudColoured::Ptr build_cloud(
    const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPose3D & pose_in_map) const;

private:
  KeyframeMapParams params_;
  mrpt::containers::yaml pipeline_;
  std::size_t num_considered_ = 0;
  std::size_t num_points_ = 0;

  struct KeyframeCloud
  {
    std::size_t index = 0;
    mrpt::viz::CPointCloudColoured::Ptr cloud;
    mrpt::viz::CSetOfObjects::Ptr node;
    mrpt::poses::CPose3D drawn_pose;
  };
  std::vector<KeyframeCloud> clouds_;
};

/** Glides scene nodes from their old to their new poses (translation and
 *  rotation interpolated, cosine easing), so map corrections read as a motion
 *  instead of a jump. Changes below MIN_GLIDE_SHIFT / MIN_GLIDE_ROTATION (as
 *  every solve brings) are applied at once. A node gliding gets a small new
 *  target without restarting its glide; a large one starts a new glide from
 *  where it is (one not started yet, e.g. waiting for the camera, keeps
 *  waiting). Times in seconds of any clock. Not thread-safe. */
class NodeAnimator
{
public:
  static constexpr double MIN_GLIDE_SHIFT = 0.01;     //!< [m]
  static constexpr double MIN_GLIDE_ROTATION = 1e-3;  //!< [rad]

  /** Starts gliding `changes` over `duration` seconds, from time `start`
   *  (nodes keep their pose until then). */
  void add(
    const std::vector<KeyframeMapVisualization::NodePose> & changes, double start, double duration,
    double now);

  /// The poses to set at time `now` (finished glides are given their final
  /// pose once, then dropped).
  [[nodiscard]] std::vector<KeyframeMapVisualization::NodePose> step(double now);

  [[nodiscard]] bool empty() const { return glides_.empty(); }
  [[nodiscard]] std::size_t size() const { return glides_.size(); }

private:
  struct Glide
  {
    mrpt::viz::CSetOfObjects::Ptr node;
    mrpt::poses::CPose3D from;
    mrpt::poses::CPose3D to;
    double start = 0;
    double duration = 0;

    [[nodiscard]] mrpt::poses::CPose3D at(double t) const;
  };
  std::unordered_map<const mrpt::viz::CSetOfObjects *, Glide> glides_;
};

/** The region to show of a map correction: around the node that moved most,
 *  the nodes within `radius` [m] of it that moved at least half as much, at
 *  their old and new positions. None if no node moved `min_shift` [m]. */
[[nodiscard]] std::optional<mrpt::math::TBoundingBox> correction_region(
  const std::vector<KeyframeMapVisualization::NodePose> & changes, double min_shift, double radius,
  double * max_shift = nullptr);

}  // namespace mola::mapper
