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
 * @file   GraphVisualization.h
 * @brief  3D scene of the mapper keyframe graph, for the GUI.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mola::mapper
{
/// What the GUI shows of the keyframe graph, and how.
struct GraphVisualizationParams
{
  /// Keyframes as spheres of this radius [m] (0: not shown).
  float keyframe_sphere_radius = 0.10f;
  /// Edges between consecutive keyframes, line width [px] (0: not shown).
  float odometry_edge_width = 2.0f;
  /// Accepted loop closures, line width [px] (0: not shown).
  float loop_edge_width = 4.0f;
  /** Loop edges drawn as a bridge this high [m] over their keyframes (0: a
   *  straight line): once a loop closes, its two keyframes overlap. */
  float loop_edge_lift = 1.0f;
  /// Loop candidates rejected by the odometry or the consistency check.
  bool show_rejected_loops = false;
  float rejected_loop_edge_width = 1.0f;

  mrpt::img::TColor keyframe_color{0xff, 0xa0, 0x00, 0xff};
  mrpt::img::TColor odometry_edge_color{0x40, 0x90, 0xff, 0xff};
  mrpt::img::TColor loop_edge_color{0x00, 0xff, 0x40, 0xff};
  mrpt::img::TColor rejected_by_consistency_color{0xff, 0x30, 0x30, 0xa0};
  mrpt::img::TColor rejected_by_odometry_color{0xa0, 0xa0, 0xa0, 0x80};

  /// GUI refresh period of the graph [s].
  double update_period = 1.0;
  /// Move the frame each front end draws its scene in (`<frame>` of its
  /// `<frame>_kf` keyframes) with the optimized map, so it stays aligned.
  bool align_front_end_frames = true;

  void loadFrom(const mrpt::containers::yaml & cfg);
};

/// The keyframe graph to draw, in the map frame.
struct GraphVisualizationInput
{
  std::vector<mrpt::math::TPoint3D> keyframes;
  /// Front end chain of each keyframe: consecutive ones of the same chain are
  /// joined by an odometry edge.
  std::vector<std::size_t> chain;
  using Pairs = std::vector<std::pair<std::size_t, std::size_t>>;
  Pairs loops;
  Pairs rejected_by_consistency;
  Pairs rejected_by_odometry;
};

/** The scene: a CSetOfObjects with the keyframe spheres, then the odometry
 *  edges, accepted loops and, if enabled, rejected loop candidates (each one a
 *  CSetOfLines, so their line widths and colors differ; loop edges with three
 *  segments each if loop_edge_lift > 0). Pairs out of range are skipped. */
[[nodiscard]] mrpt::viz::CSetOfObjects::Ptr build_graph_visualization(
  const GraphVisualizationInput & in, const GraphVisualizationParams & p);

/** The scene frame a front end draws in, from the source frame of its
 *  keyframes: "<f>" for "<f>_kf" (as LIO does), none otherwise. */
[[nodiscard]] std::optional<std::string> front_end_viz_frame(const std::string & sourceFrame);

/// A keyframe pose in the map and in the frame of its front end.
struct FrontEndKeyframePose
{
  std::string source_frame;
  mrpt::poses::CPose3D pose_in_map;
  mrpt::poses::CPose3D pose_in_source;
};

/** Pose in the map of the scene frame of each front end (front_end_viz_frame()),
 *  T_map_f = pose_in_map (+) inverse(pose_in_source), from the given keyframes
 *  (later ones win: pass the newest of each chain last). Keyframes of source
 *  frames without a scene frame are ignored. */
[[nodiscard]] std::map<std::string, mrpt::poses::CPose3D> front_end_viz_frame_poses(
  const std::vector<FrontEndKeyframePose> & keyframes);

}  // namespace mola::mapper
