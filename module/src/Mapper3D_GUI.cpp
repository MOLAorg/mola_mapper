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

#include <algorithm>
#include <cmath>
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
  bool estimatingGeoref = false;
  std::size_t gnssFactors = 0;
  std::size_t imuFactors = 0;
  std::optional<mrpt::topography::TGeodeticCoords> tentativeGeo;
  std::optional<mrpt::poses::CPose3DPDFGaussian> enuToMap;

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
    estimatingGeoref = params_.estimate_geo_reference || params_.fixed_geo_reference.has_value();
    gnssFactors = gnss_factors_inserted_;
    imuFactors = imu_factors_inserted_;
    tentativeGeo = state_.tentative_geo_coord_reference;
    if (const auto itEnu = state_.last_estimated_frames.find(0);
        itEnu != state_.last_estimated_frames.end()) {
      enuToMap = itEnu->second;
    }
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

    // The {enu} geo-reference frame (id 0 = T_enu_to_map). It is skipped in the
    // odomFrames loop above (it is not an odometry source), but the user still
    // wants to SEE where ENU sits relative to {map}. last_estimated_frames[0]
    // stores T_enu_to_map (pose of {map} in {enu}), so the ENU origin in {map}
    // is its inverse. Drawn only while geo-referencing (otherwise it is just the
    // weak-prior identity). A larger, distinct-color corner + "enu" label.
    if (estimatingGeoref && enuToMap.has_value() && viz_show_odom_frames_.load()) {
      const mrpt::poses::CPose3D enuInMap = mrpt::poses::CPose3D() - enuToMap->mean;
      auto glEnu = mrpt::opengl::stock_objects::CornerXYZ(2.0f);
      glEnu->setPose(enuInMap);
      glMarkers->insert(glEnu);

      auto glEnuText = mrpt::opengl::CText::Create("enu");
      glEnuText->setColor_u8(0x00, 0xff, 0xff, 0xff);  // cyan, distinct from odom
      glEnuText->setPose(enuInMap);
      glMarkers->insert(glEnuText);
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

    // GNSS + tentative ENU origin:
    if (tentativeGeo.has_value()) {
      gui_.lbGnss->set(mrpt::format(
        "GNSS factors: %zu  origin(%.5f, %.5f)", gnssFactors, tentativeGeo->lat.decimal_value,
        tentativeGeo->lon.decimal_value));
    } else {
      gui_.lbGnss->set(mrpt::format("GNSS factors: %zu  (no fix yet)", gnssFactors));
    }

    // T_enu_to_map estimate (the geo-referencing transform) + its sigmas:
    if (enuToMap.has_value()) {
      const auto & m = enuToMap->mean;
      const double sPos =
        std::sqrt(std::max({enuToMap->cov(0, 0), enuToMap->cov(1, 1), enuToMap->cov(2, 2)}));
      const double sYawDeg = mrpt::RAD2DEG(std::sqrt(enuToMap->cov(3, 3)));
      gui_.lbEnu->set(mrpt::format(
        "T_enu_map: (%.1f,%.1f,%.1f) yaw=%.1fdeg s_pos=%.2fm s_yaw=%.2fdeg", m.x(), m.y(), m.z(),
        mrpt::RAD2DEG(m.yaw()), sPos, sYawDeg));
    } else {
      gui_.lbEnu->set("T_enu_map: (not estimated)");
    }

    // Per-source odom-frame correction vs {map}: T_map_to_odom_i as a
    // translation magnitude AND a rotation magnitude. This is how far each front
    // end's {odom_i} frame has been pulled to absorb its own drift; for LIO the
    // characteristic drift is z/tilt, so the ROTATION term is the informative
    // one (the translation often stays near zero because the tight
    // consecutive-keyframe edges keep {map} and {odom} aligned in position).
    // Note: a SharedKeyframeMap sink frame (e.g. "odom_kf") is anchored to the
    // first keyframe only, so it stays ~0 by design; the dense fuse_pose() frame
    // (e.g. "odom") is the one that tracks ongoing drift.
    std::string driftStr = "Odom drift:";
    if (odomFrames.empty()) {
      driftStr += " (none)";
    }
    for (const auto & [name, pose] : odomFrames) {
      // Rotation magnitude (geodesic angle) of T_map_to_odom_i from its rotation
      // matrix: angle = acos((trace(R) - 1) / 2).
      const double cosAngle = std::clamp((pose.getRotationMatrix().trace() - 1.0) * 0.5, -1.0, 1.0);
      const double angleDeg = mrpt::RAD2DEG(std::acos(cosAngle));
      driftStr +=
        mrpt::format(" %s=%.2fm/%.1fdeg", name.c_str(), pose.translation().norm(), angleDeg);
    }
    gui_.lbDrift->set(driftStr);
    gui_.lbImu->set(mrpt::format("IMU factors: %zu", imuFactors));
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
  gui_.lbGnss = std::make_shared<LiveString>(" ");
  gui_.lbEnu = std::make_shared<LiveString>(" ");
  gui_.lbDrift = std::make_shared<LiveString>(" ");
  gui_.lbImu = std::make_shared<LiveString>(" ");

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
    tab.widgets.emplace_back(Label{gui_.lbImu});
    tab.widgets.emplace_back(Label{gui_.lbConverged});
    desc.tabs.emplace_back(std::move(tab));
  }

  // Geo-referencing tab:
  {
    Tab tab;
    tab.title = "Geo-ref";
    tab.widgets.emplace_back(Label{gui_.lbGeoref});
    tab.widgets.emplace_back(Label{gui_.lbGnss});
    tab.widgets.emplace_back(Label{gui_.lbEnu});
    tab.widgets.emplace_back(Label{gui_.lbDrift});
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
