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
 * @file   GraphVisualization.cpp
 * @brief  3D scene of the mapper keyframe graph, for the GUI.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GraphVisualization.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSphere.h>

#include <string_view>

namespace mola::mapper
{
namespace
{
// Few polygons: a map may have thousands of keyframes.
constexpr int SPHERE_DIVISIONS = 8;

mrpt::viz::CSetOfLines::Ptr make_lines(
  const std::vector<mrpt::math::TPoint3D> & kfs, const GraphVisualizationInput::Pairs & pairs,
  float width, const mrpt::img::TColor & color, float lift = 0)
{
  auto gl = mrpt::viz::CSetOfLines::Create();
  gl->setLineWidth(width);
  gl->setColor_u8(color);
  gl->reserve(lift > 0 ? 3 * pairs.size() : pairs.size());
  for (const auto & [a, b] : pairs) {
    if (a >= kfs.size() || b >= kfs.size()) {
      continue;
    }
    if (lift <= 0) {
      gl->appendLine(kfs[a], kfs[b]);
      continue;
    }
    const mrpt::math::TPoint3D up(0, 0, lift);
    gl->appendLine(kfs[a], kfs[a] + up);
    gl->appendLine(kfs[a] + up, kfs[b] + up);
    gl->appendLine(kfs[b] + up, kfs[b]);
  }
  return gl;
}
}  // namespace

void GraphVisualizationParams::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, keyframe_sphere_radius);
  MCP_LOAD_OPT(cfg, odometry_edge_width);
  MCP_LOAD_OPT(cfg, loop_edge_width);
  MCP_LOAD_OPT(cfg, loop_edge_lift);
  MCP_LOAD_OPT(cfg, show_rejected_loops);
  MCP_LOAD_OPT(cfg, rejected_loop_edge_width);
  MCP_LOAD_OPT(cfg, update_period);
  MCP_LOAD_OPT(cfg, align_front_end_frames);
}

mrpt::viz::CSetOfObjects::Ptr build_graph_visualization(
  const GraphVisualizationInput & in, const GraphVisualizationParams & p)
{
  auto scene = mrpt::viz::CSetOfObjects::Create();
  const auto & kfs = in.keyframes;

  if (p.keyframe_sphere_radius > 0) {
    for (const auto & pt : kfs) {
      auto glSphere = mrpt::viz::CSphere::Create(p.keyframe_sphere_radius, SPHERE_DIVISIONS);
      glSphere->setColor_u8(p.keyframe_color);
      glSphere->setLocation(pt);
      scene->insert(glSphere);
    }
  }

  if (p.odometry_edge_width > 0) {
    GraphVisualizationInput::Pairs chainEdges;
    for (std::size_t i = 1; i < kfs.size(); i++) {
      if (i < in.chain.size() && in.chain[i] == in.chain[i - 1]) {
        chainEdges.emplace_back(i - 1, i);
      }
    }
    scene->insert(make_lines(kfs, chainEdges, p.odometry_edge_width, p.odometry_edge_color));
  }

  if (p.loop_edge_width > 0) {
    scene->insert(
      make_lines(kfs, in.loops, p.loop_edge_width, p.loop_edge_color, p.loop_edge_lift));
  }

  if (p.show_rejected_loops && p.rejected_loop_edge_width > 0) {
    scene->insert(make_lines(
      kfs, in.rejected_by_consistency, p.rejected_loop_edge_width, p.rejected_by_consistency_color,
      p.loop_edge_lift));
    scene->insert(make_lines(
      kfs, in.rejected_by_odometry, p.rejected_loop_edge_width, p.rejected_by_odometry_color,
      p.loop_edge_lift));
  }
  return scene;
}

std::optional<std::string> front_end_viz_frame(const std::string & sourceFrame)
{
  constexpr std::string_view suffix = "_kf";
  if (
    sourceFrame.size() <= suffix.size() ||
    sourceFrame.compare(sourceFrame.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return std::nullopt;
  }
  return sourceFrame.substr(0, sourceFrame.size() - suffix.size());
}

std::map<std::string, mrpt::poses::CPose3D> front_end_viz_frame_poses(
  const std::vector<FrontEndKeyframePose> & keyframes)
{
  std::map<std::string, mrpt::poses::CPose3D> frames;
  for (const auto & kf : keyframes) {
    if (const auto f = front_end_viz_frame(kf.source_frame); f) {
      frames[*f] = kf.pose_in_map + (mrpt::poses::CPose3D() - kf.pose_in_source);
    }
  }
  return frames;
}

}  // namespace mola::mapper
