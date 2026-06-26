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
 * @file   Mapper3D_GUI.cpp
 * @brief  Visualization (MolaViz / MolaVizImGui) for the central 3D map:
 *         keyframe tree, graph edges, and per-source movable {odom_i} frames.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/datetime.h>

#include <set>
#include <utility>

namespace mola::mapper_3d
{

void Mapper3D::updateVisualization()
{
  if (!visualizer_) {
    return;
  }

  // Throttle to the configured rate (default 5 Hz) using wall-clock time:
  double updateRateHz = 5.0;
  if (viz_params_.has("update_rate_hz")) {
    updateRateHz = viz_params_["update_rate_hz"].as<double>();
  }
  if (updateRateHz <= 0) {
    return;  // visualization explicitly disabled
  }
  const auto nowWall = mrpt::Clock::now();
  if (last_viz_update_wallclock_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_viz_update_wallclock_, nowWall);
    if (dt < 1.0 / updateRateHz) {
      return;
    }
  }
  last_viz_update_wallclock_ = nowWall;

  const float kfCornerSize = viz_params_.getOrDefault<float>("keyframe_corner_size", 0.5f);

  // Snapshot everything we need under the state lock, then release it before
  // talking to the visualizer (which only enqueues GUI-thread tasks anyway).
  std::vector<std::pair<KeyFrameID, mrpt::poses::CPose3D>> kfPoses;
  std::vector<std::pair<mrpt::math::TPoint3D, mrpt::math::TPoint3D>> edges;
  std::vector<std::pair<std::string, mrpt::poses::CPose3D>> odomFrames;
  std::optional<mrpt::poses::CPose3D> latestVehiclePose;
  bool hasGeoref = false;

  {
    auto lck = mrpt::lockHelper(stateMutex_);

    // Keyframe poses, in {map}, ordered by time:
    for (const auto & [t, kfId] : state_.time_to_kf_id.getDirectMap()) {
      (void)t;
      const auto it = state_.last_estimated_states.find(kfId);
      if (it == state_.last_estimated_states.end()) {
        continue;
      }
      kfPoses.emplace_back(kfId, it->second.pose);
    }
    if (!kfPoses.empty()) {
      latestVehiclePose = kfPoses.back().second;
    }

    // Graph edges (undirected): de-duplicate the (a,b)/(b,a) pairs.
    std::set<std::pair<KeyFrameID, KeyFrameID>> seen;
    for (const auto & [pair, data] : state_.kf_connectivity.edges) {
      (void)data;
      const auto a = std::min(pair.first, pair.second);
      const auto b = std::max(pair.first, pair.second);
      if (a == b || !seen.insert({a, b}).second) {
        continue;
      }
      const auto itA = state_.last_estimated_states.find(a);
      const auto itB = state_.last_estimated_states.find(b);
      if (itA == state_.last_estimated_states.end() || itB == state_.last_estimated_states.end()) {
        continue;
      }
      edges.emplace_back(itA->second.pose.translation(), itB->second.pose.translation());
    }

    // Per-source frames: T_map_to_odom_i (skip id 0 = ENU / T_enu_to_map).
    const auto & idToName = state_.known_odom_frames.getInverseMap();
    for (const auto & [id, framePdf] : state_.last_estimated_frames) {
      if (id == 0) {
        continue;
      }
      const auto itName = idToName.find(id);
      if (itName == idToName.end()) {
        continue;
      }
      odomFrames.emplace_back(itName->second, framePdf.mean);
    }

    hasGeoref = state_.geo_reference.has_value();
  }

  // --- Keyframe tree (corners + trajectory polyline) ---
  {
    auto glKfs = mrpt::opengl::CSetOfObjects::Create();
    if (viz_show_keyframes_.load()) {
      auto glPath = mrpt::opengl::CSetOfLines::Create();
      glPath->setColor_u8(0x00, 0xc0, 0x00, 0xff);  // green trajectory
      bool first = true;
      mrpt::math::TPoint3D prev;
      for (const auto & [kfId, pose] : kfPoses) {
        (void)kfId;
        if (kfCornerSize > 0) {
          auto glCorner = mrpt::opengl::stock_objects::CornerXYZSimple(kfCornerSize);
          glCorner->setPose(pose);
          glKfs->insert(glCorner);
        }
        const auto pt = pose.translation();
        if (first) {
          first = false;
        } else {
          glPath->appendLine(prev, pt);
        }
        prev = pt;
      }
      glKfs->insert(glPath);
    }
    visualizer_->update_3d_object("mapper3d/keyframes", glKfs);
  }

  // --- Graph edges ---
  {
    auto glEdges = mrpt::opengl::CSetOfObjects::Create();
    if (viz_show_edges_.load()) {
      auto glLines = mrpt::opengl::CSetOfLines::Create();
      glLines->setColor_u8(0x40, 0x80, 0xff, 0x80);  // translucent blue
      for (const auto & [a, b] : edges) {
        glLines->appendLine(a, b);
      }
      glEdges->insert(glLines);
    }
    visualizer_->update_3d_object("mapper3d/edges", glEdges);
  }

  // --- Per-source movable frame nodes + visible markers ---
  {
    auto glMarkers = mrpt::opengl::CSetOfObjects::Create();
    for (const auto & [name, pose] : odomFrames) {
      // Functional part: reposition the movable frame node front ends draw
      // their dense clouds / local map under (see VizInterface::update_3d_object
      // parentFrame). Even with markers hidden, keep moving the frame so the
      // attached geometry stays correctly placed in {map}.
      visualizer_->update_3d_object_frame(name, pose.asTPose());

      if (!viz_show_odom_frames_.load()) {
        continue;
      }
      auto glCorner = mrpt::opengl::stock_objects::CornerXYZ(1.0f);
      glCorner->setPose(pose);
      glMarkers->insert(glCorner);

      auto glText = mrpt::opengl::CText::Create(name);
      glText->setColor_u8(0xff, 0xff, 0x00, 0xff);
      glText->setPose(pose);
      glMarkers->insert(glText);
    }
    visualizer_->update_3d_object("mapper3d/odom_frames", glMarkers);
  }

  // --- Camera follow ---
  if (viz_camera_follows_vehicle_.load() && latestVehiclePose.has_value()) {
    visualizer_->update_viewport_look_at(mrpt::math::TPoint3Df(
      static_cast<float>(latestVehiclePose->x()), static_cast<float>(latestVehiclePose->y()),
      static_cast<float>(latestVehiclePose->z())));
  }

  // --- GUI sub-window (created once) + text labels ---
  {
    auto lckGui = mrpt::lockHelper(state_gui_mtx_);
    if (!gui_created_) {
      internalBuildGUI();
      gui_created_ = true;
    }
  }

  if (gui_.lbKeyframes) {
    gui_.lbKeyframes->set(mrpt::format("Keyframes: %zu", kfPoses.size()));
    gui_.lbEdges->set(mrpt::format("Graph edges: %zu", edges.size()));
    gui_.lbOdomFrames->set(mrpt::format("Odometry frames: %zu", odomFrames.size()));
    gui_.lbGeoref->set(std::string("Geo-referenced: ") + (hasGeoref ? "yes" : "no"));

    mrpt::poses::CPose3DPDFGaussian convPose;
    const bool converged = has_converged_localization(convPose);
    gui_.lbConverged->set(std::string("Localization: ") + (converged ? "converged" : "(no)"));
  }
}

void Mapper3D::internalBuildGUI()
{
  using namespace mola::gui;

  // LiveStrings shared between this module (writer) and the GUI (reader):
  gui_.lbKeyframes = std::make_shared<LiveString>(" ");
  gui_.lbEdges = std::make_shared<LiveString>(" ");
  gui_.lbOdomFrames = std::make_shared<LiveString>(" ");
  gui_.lbGeoref = std::make_shared<LiveString>(" ");
  gui_.lbConverged = std::make_shared<LiveString>(" ");

  WindowDescription desc;
  desc.title = "mola_mapper_3d";
  desc.position = {5, 700};
  desc.size = {320, 0};

  // Status tab:
  {
    Tab tab;
    tab.title = "Status";
    tab.widgets.emplace_back(Label{gui_.lbKeyframes});
    tab.widgets.emplace_back(Label{gui_.lbEdges});
    tab.widgets.emplace_back(Label{gui_.lbOdomFrames});
    tab.widgets.emplace_back(Label{gui_.lbGeoref});
    tab.widgets.emplace_back(Label{gui_.lbConverged});
    desc.tabs.emplace_back(std::move(tab));
  }

  // View tab:
  {
    Tab tab;
    tab.title = "View";
    tab.widgets.emplace_back(CheckBox{
      "Show keyframes", viz_show_keyframes_.load(),
      [this](bool checked) { viz_show_keyframes_.store(checked); }});
    tab.widgets.emplace_back(CheckBox{
      "Show graph edges", viz_show_edges_.load(),
      [this](bool checked) { viz_show_edges_.store(checked); }});
    tab.widgets.emplace_back(CheckBox{
      "Show odometry frames", viz_show_odom_frames_.load(),
      [this](bool checked) { viz_show_odom_frames_.store(checked); }});
    tab.widgets.emplace_back(CheckBox{
      "Camera follows vehicle", viz_camera_follows_vehicle_.load(),
      [this](bool checked) { viz_camera_follows_vehicle_.store(checked); }});
    desc.tabs.emplace_back(std::move(tab));
  }

  visualizer_->create_subwindow_from_description(desc).get();
}

}  // namespace mola::mapper_3d
