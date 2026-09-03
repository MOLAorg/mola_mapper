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
 * @file   Mapper_GUI.cpp
 * @brief  Visualization (MolaViz / MolaVizImGui) for the central 3D map:
 *         keyframe tree, graph edges, and per-source movable {odom_i} frames.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/datetime.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/stock_objects.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <type_traits>
#include <utility>

namespace mola::mapper
{

namespace
{
// Preset values offered by the "Corner scale" / "Sphere radius" combo boxes in
// the GUI "View" tab (index-aligned with their label lists below). 0 = hidden.
constexpr std::array<float, 7> kKeyframeCornerScalePresets{0.0f, 0.1f, 0.2f, 0.3f,
                                                           0.5f, 1.0f, 2.0f};
constexpr std::array<float, 6> kKeyframeSphereRadiusPresets{0.0f, 0.1f, 0.2f, 0.3f, 0.5f, 1.0f};
constexpr std::array<float, 6> kEdgeCylinderRadiusPresets{0.02f, 0.05f, 0.1f, 0.2f, 0.3f, 0.5f};
// Fixed pixel width for the combo boxes above, so their (short) option labels
// don't stretch to fill the sub-window as it is resized (ImGui's default
// combo width tracks the available content width).
constexpr int kComboFixedWidthPx = 90;

// Graph edges render as CCylinder (thin tubes), visible even at a distance,
// unlike GL lines. Few radial slices and no end caps keep the per-edge
// triangle count low: a dense keyframe graph can have thousands of edges.
constexpr uint32_t kEdgeCylinderSlices = 6;

// mola::gui::ComboBox::fixed_width was added after this file's minimum
// supported mola_kernel; detect it at compile time (rather than pinning a
// mola_kernel version number) so this still builds against older headers.
template <typename T, typename = void>
struct combo_has_fixed_width : std::false_type
{
};
template <typename T>
struct combo_has_fixed_width<T, std::void_t<decltype(std::declval<T &>().fixed_width)>>
: std::true_type
{
};

template <typename ComboBoxT>
void setComboFixedWidthIfSupported(ComboBoxT & combo, int width)
{
  if constexpr (combo_has_fixed_width<ComboBoxT>::value) {
    combo.fixed_width = width;
  } else {
    (void)combo;
    (void)width;
  }
}

template <std::size_t N>
int closestPresetIndex(const std::array<float, N> & presets, float value)
{
  int best = 0;
  float bestDiff = std::abs(presets[0] - value);
  for (std::size_t i = 1; i < N; i++) {
    const float diff = std::abs(presets[i] - value);
    if (diff < bestDiff) {
      bestDiff = diff;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// A CCylinder's local Z axis runs from its base (at the object's origin) to
// its top (at height `len`). Orient it so that axis points from `a` to `b`:
// pitch = acos(nz), yaw = atan2(ny, nx) for the unit direction (nx,ny,nz),
// derived from MRPT's R = Rz(yaw)*Ry(pitch)*Rx(roll) convention applied to
// local Z (roll is a free rotation about the cylinder's own axis, so 0 is
// fine). Returns nullptr for a degenerate (near-zero-length) edge.
mrpt::viz::CCylinder::Ptr makeEdgeCylinder(
  const mrpt::math::TPoint3D & a, const mrpt::math::TPoint3D & b, float radius)
{
  const mrpt::math::TPoint3D d = b - a;
  const double len = d.norm();
  if (len < 1e-6) {
    return nullptr;
  }
  const double pitch = std::acos(std::clamp(d.z / len, -1.0, 1.0));
  const double yaw = std::atan2(d.y, d.x);

  auto glCyl =
    mrpt::viz::CCylinder::Create(radius, radius, static_cast<float>(len), kEdgeCylinderSlices);
  glCyl->setHasBases(false, false);
  glCyl->setPose(mrpt::poses::CPose3D(a.x, a.y, a.z, yaw, pitch, 0.0));
  return glCyl;
}
}  // namespace

void Mapper::updateVisualization()
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

  // Camera-follow runs on EVERY tick (it self-throttles to a high rate), before
  // the scene-rebuild throttle below, so the camera tracks the dense vehicle
  // pose smoothly instead of only refreshing at the (slower) scene rate.
  updateCameraFollow();

  const auto nowWall = mrpt::Clock::now();
  if (last_viz_update_wallclock_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_viz_update_wallclock_, nowWall);
    if (dt < 1.0 / updateRateHz) {
      return;
    }
  }
  last_viz_update_wallclock_ = nowWall;

  const float kfCornerScale = viz_keyframe_corner_scale_.load();
  const float kfSphereRadius = viz_keyframe_sphere_radius_.load();

  // Snapshot everything we need under the state lock, then release it before
  // talking to the visualizer (which only enqueues GUI-thread tasks anyway).
  std::vector<std::pair<KeyFrameID, mrpt::poses::CPose3D>> kfPoses;
  std::vector<std::pair<mrpt::math::TPoint3D, mrpt::math::TPoint3D>> edges;
  std::vector<std::pair<mrpt::math::TPoint3D, mrpt::math::TPoint3D>> lcEdges;
  std::vector<std::pair<std::string, mrpt::poses::CPose3D>> odomFrames;
  bool hasGeoref = false;
  std::size_t gnssFactors = 0;
  std::size_t imuFactorsGravity = 0;
  std::size_t imuFactorsAttitude = 0;
  std::size_t imuFactorsOmega = 0;
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

    // Graph edges (undirected): de-duplicate the (a,b)/(b,a) pairs, and split
    // loop-closure edges (present in the LC merged-pairs set) from the
    // consecutive-keyframe odometry chain so they can be drawn distinctly.
    const bool highlightLc = viz_highlight_lc_edges_.load();
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
      const bool isLc = highlightLc && lc_scan_.merged_pairs.count(std::minmax(a, b)) != 0;
      auto & target = isLc ? lcEdges : edges;
      target.emplace_back(itA->second.pose.translation(), itB->second.pose.translation());
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
    gnssFactors = geo_ref_counters_.gnss;
    imuFactorsGravity = geo_ref_counters_.imu_gravity;
    imuFactorsAttitude = geo_ref_counters_.imu_attitude;
    imuFactorsOmega = geo_ref_counters_.imu_omega;
    tentativeGeo = state_.tentative_geo_coord_reference;
    if (const auto itEnu = state_.last_estimated_frames.find(0);
        itEnu != state_.last_estimated_frames.end()) {
      enuToMap = itEnu->second;
    }
  }  // end of stateMutex_ lock

  // Viz reference frame: the transform that maps {map}-frame coordinates into
  // the scene origin selected by the user (View tab). For {map} it is identity;
  // for {enu} it is T_enu_to_map (so a map pose p renders at T_enu_to_map (+) p,
  // i.e. in {enu}, North-oriented). {enu} always exists: if no geo-ref has been
  // estimated, T_enu_to_map is the identity weak-prior, so the scene is
  // unchanged. All top-level scene objects are placed under this transform.
  mrpt::poses::CPose3D vizXform;  // identity by default ({map})
  if (viz_reference_frame_.load() == 1 && enuToMap.has_value()) {
    vizXform = enuToMap->mean;
  }

  // --- Keyframe tree (corners + trajectory polyline) ---
  {
    auto glKfs = mrpt::viz::CSetOfObjects::Create();
    if (viz_show_keyframes_.load()) {
      auto glPath = mrpt::viz::CSetOfLines::Create();
      glPath->setColor_u8(0x00, 0xc0, 0x00, 0xff);  // green trajectory
      bool first = true;
      mrpt::math::TPoint3D prev;
      for (const auto & [kfId, pose] : kfPoses) {
        (void)kfId;
        if (kfCornerScale > 0) {
          auto glCorner = mrpt::viz::stock_objects::CornerXYZ(kfCornerScale);
          glCorner->setPose(pose);
          glKfs->insert(glCorner);
        }
        if (kfSphereRadius > 0) {
          auto glSphere = mrpt::viz::CSphere::Create(kfSphereRadius, 10 /*few polygons*/);
          glSphere->setColor_u8(0xff, 0xa0, 0x00, 0xff);  // orange
          glSphere->setLocation(pose.translation());
          glKfs->insert(glSphere);
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
    glKfs->setPose(vizXform);
    visualizer_->update_3d_object("mapper/keyframes", glKfs);
  }

  // --- Graph edges (rendered as thin CCylinder tubes, not GL lines: lines are
  //     barely visible at typical zoom levels / with anti-aliasing off) ---
  {
    auto glEdges = mrpt::viz::CSetOfObjects::Create();
    if (viz_show_edges_.load()) {
      const float edgeRadius = viz_edge_cylinder_radius_.load();
      for (const auto & [a, b] : edges) {
        auto glCyl = makeEdgeCylinder(a, b, edgeRadius);
        if (!glCyl) {
          continue;
        }
        glCyl->setColor_u8(0x40, 0x80, 0xff, 0xc0);  // translucent blue (odometry chain)
        glEdges->insert(glCyl);
      }
      // Loop-closure edges: brighter green and thicker, so accepted loops stand
      // out against the odometry chain (they physically stitch distant legs).
      for (const auto & [a, b] : lcEdges) {
        auto glCyl = makeEdgeCylinder(a, b, edgeRadius * 2.0f);
        if (!glCyl) {
          continue;
        }
        glCyl->setColor_u8(0x00, 0xe0, 0x00, 0xff);  // opaque green (loop closure)
        glEdges->insert(glCyl);
      }
    }
    glEdges->setPose(vizXform);
    visualizer_->update_3d_object("mapper/edges", glEdges);
  }

  // --- Ground grid (XY extent from the trajectory bounding box) ---
  {
    auto glGroundGrid = mrpt::viz::CSetOfObjects::Create();
    if (viz_show_ground_grid_.load()) {
      const float spacing = viz_params_.getOrDefault<float>("ground_grid_spacing", 10.0f);

      float xMin = -50.0f, xMax = 50.0f, yMin = -50.0f, yMax = 50.0f;
      if (!kfPoses.empty()) {
        xMin = xMax = static_cast<float>(kfPoses.front().second.x());
        yMin = yMax = static_cast<float>(kfPoses.front().second.y());
        for (const auto & [kfId, pose] : kfPoses) {
          const float x = static_cast<float>(pose.x());
          const float y = static_cast<float>(pose.y());
          xMin = std::min(xMin, x);
          xMax = std::max(xMax, x);
          yMin = std::min(yMin, y);
          yMax = std::max(yMax, y);
        }
        xMin -= spacing;
        xMax += spacing;
        yMin -= spacing;
        yMax += spacing;
      }

      auto glGrid = mrpt::viz::CGridPlaneXY::Create();
      glGrid->setPlaneLimits(xMin, xMax, yMin, yMax);
      glGrid->setGridFrequency(spacing);
      glGrid->setColor_u8(0xff, 0xff, 0xff, 0x80);
      glGroundGrid->insert(glGrid);
    }
    glGroundGrid->setPose(vizXform);
    visualizer_->update_3d_object("mapper/groundgrid", glGroundGrid);
  }

  // --- Per-source movable frame nodes + visible markers ---
  {
    auto glMarkers = mrpt::viz::CSetOfObjects::Create();
    for (const auto & [name, pose] : odomFrames) {
      // Functional part: reposition the movable frame node front ends draw
      // their dense clouds / local map under (see VizInterface::update_3d_object
      // parentFrame). Even with markers hidden, keep moving the frame so the
      // attached geometry stays correctly placed in the selected viz frame
      // (compose the per-source T_map_to_odom_i with the scene viz transform).
      visualizer_->update_3d_object_frame(name, (vizXform + pose).asTPose());

      if (!viz_show_odom_frames_.load()) {
        continue;
      }
      auto glCorner = mrpt::viz::stock_objects::CornerXYZ(1.0f);
      glCorner->setPose(pose);
      glMarkers->insert(glCorner);

      auto glText = mrpt::viz::CText::Create(name);
      glText->setColor_u8(0xff, 0xff, 0x00, 0xff);
      glText->setPose(pose);
      glMarkers->insert(glText);
    }

    // The {enu} XYZ corner (id 0 = T_enu_to_map). Shown only when {enu} is the
    // selected viz reference frame (the user's "optionally visible" = only in
    // ENU view). The corner is placed at the ENU origin: in the glMarkers local
    // ({map}) coordinates that is inv(T_enu_to_map), which the container's
    // vizXform (= T_enu_to_map) then maps back to the scene origin. {enu} always
    // exists (identity until geo-ref converges), so this works with or without
    // GNSS. A larger, distinct-color corner + "enu" label.
    if (viz_reference_frame_.load() == 1) {
      const mrpt::poses::CPose3D enuInMap =
        enuToMap.has_value() ? (mrpt::poses::CPose3D() - enuToMap->mean) : mrpt::poses::CPose3D();
      auto glEnu = mrpt::viz::stock_objects::CornerXYZ(2.0f);
      glEnu->setPose(enuInMap);
      glMarkers->insert(glEnu);

      auto glEnuText = mrpt::viz::CText::Create("enu");
      glEnuText->setColor_u8(0x00, 0xff, 0xff, 0xff);  // cyan, distinct from odom
      glEnuText->setPose(enuInMap);
      glMarkers->insert(glEnuText);
    }
    glMarkers->setPose(vizXform);
    visualizer_->update_3d_object("mapper/odom_frames", glMarkers);
  }

  // Camera-follow is handled by updateCameraFollow() at the top of this method
  // (it runs at a higher rate than this throttled scene rebuild).

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
    gui_.lbEdges->set(mrpt::format("Graph edges: %zu", edges.size() + lcEdges.size()));
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
    // Note: in SharedMapOnly mode the SharedKeyframeMap source ("<name>_kf") is
    // the only one that creates keyframes, so it is where T_map_to_odom_i is
    // computed; optimize_and_refresh() then republishes it under the base frame
    // ("<name>", the publish_reference_frame the front end draws / queries
    // under), which is what appears here and what the movable viz frame uses.
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
    gui_.lbImu->set(mrpt::format(
      "IMU factors: grav=%zu att=%zu omega=%zu", imuFactorsGravity, imuFactorsAttitude,
      imuFactorsOmega));
  }

  // --- Loop-closure tab (present only when loop_closure_enabled) ---
  if (gui_.lbLcLoops) {
    gui_.lbLcLoops->set(mrpt::format("Loops accepted: %zu", lc_ui_.loops_accepted.load()));
    gui_.lbLcCandidates->set(mrpt::format(
      "Candidates checked: %zu over %zu scan(s)", lc_ui_.candidates_checked.load(),
      lc_ui_.scans_completed.load()));

    if (lc_ui_.finalize_active.load()) {
      gui_.lbLcCurrent->set(mrpt::format(
        "Current: finalize round %zu/%zu", lc_ui_.finalize_round.load(),
        lc_ui_.finalize_rounds_total.load()));
    } else if (lc_ui_.scan_in_progress.load()) {
      const std::size_t total = lc_ui_.cur_total.load();
      const std::size_t done = lc_ui_.cur_done.load();
      const std::size_t pending = total > done ? total - done : 0;
      gui_.lbLcCurrent->set(
        mrpt::format("Current scan: %zu/%zu evaluated, %zu pending", done, total, pending));
    } else {
      gui_.lbLcCurrent->set("Current scan: idle");
    }

    gui_.lbLcLast->set(mrpt::format(
      "Last scan: +%zu loop(s), %.1f s (%s)", lc_ui_.last_scan_accepted.load(),
      lc_ui_.last_scan_seconds.load(), lc_ui_.last_scan_full.load() ? "full" : "incremental"));
  }
}

void Mapper::updateCameraFollow()
{
  if (!visualizer_ || !viz_camera_follows_vehicle_.load()) {
    return;
  }

  // Self-throttle to a smooth-but-cheap wall-clock rate. This runs every
  // spinOnce (well above the scene rate), so without this it would enqueue a
  // look-at every tick.
  constexpr double kCameraFollowRateHz = 30.0;
  const auto nowWall = mrpt::Clock::now();
  if (last_camera_follow_wallclock_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_camera_follow_wallclock_, nowWall);
    if (dt < 1.0 / kCameraFollowRateHz) {
      return;
    }
  }

  // Snapshot the little we need under the state lock; the geo-ref transform (for
  // the {enu} viz frame) and a last-solved-keyframe fallback.
  std::optional<mrpt::Clock::time_point> freshStamp;
  std::optional<mrpt::poses::CPose3D> enuToMap;
  std::optional<mrpt::poses::CPose3D> latestKfPose;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    freshStamp = get_current_extrapolated_stamp_locked();
    if (const auto itEnu = state_.last_estimated_frames.find(0);
        itEnu != state_.last_estimated_frames.end()) {
      enuToMap = itEnu->second.mean;
    }
    if (!state_.time_to_kf_id.empty()) {
      const auto kfId = state_.time_to_kf_id.getDirectMap().rbegin()->second;
      if (const auto it = state_.last_estimated_states.find(kfId);
          it != state_.last_estimated_states.end()) {
        latestKfPose = it->second.pose;
      }
    }
  }

  // Prefer the freshest dense fuse_pose() vehicle pose (LIO pushes these every
  // scan, between the sparse keyframes), extrapolated to the current instant.
  // This tracks the actual high-rate motion. Fall back to the kinematic
  // estimate, then the last solved keyframe.
  std::optional<mrpt::poses::CPose3D> target;
  if (freshStamp.has_value()) {
    target = freshest_vehicle_pose_in_map(*freshStamp);
    if (!target.has_value()) {
      const auto nv = estimated_navstate(*freshStamp, params_.reference_frame_name);
      if (nv.has_value()) {
        target = nv->pose.getPoseMean();
      }
    }
  }
  if (!target.has_value()) {
    target = latestKfPose;
  }
  if (!target.has_value()) {
    return;
  }

  last_camera_follow_wallclock_ = nowWall;

  // Render in the selected viz frame (scene geometry is placed under vizXform:
  // identity for {map}, T_enu_to_map for {enu}).
  mrpt::poses::CPose3D vizXform;
  if (viz_reference_frame_.load() == 1 && enuToMap.has_value()) {
    vizXform = *enuToMap;
  }
  const mrpt::poses::CPose3D targetInScene = vizXform + *target;
  visualizer_->update_viewport_look_at(mrpt::math::TPoint3Df(
    static_cast<float>(targetInScene.x()), static_cast<float>(targetInScene.y()),
    static_cast<float>(targetInScene.z())));
}

void Mapper::internalBuildGUI()
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
  if (params_.loop_closure_enabled) {
    gui_.lbLcLoops = std::make_shared<LiveString>(" ");
    gui_.lbLcCandidates = std::make_shared<LiveString>(" ");
    gui_.lbLcCurrent = std::make_shared<LiveString>(" ");
    gui_.lbLcLast = std::make_shared<LiveString>(" ");
  }

  WindowDescription desc;
  desc.title = "mola_mapper";
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

  // Loop-closure tab (only when the feature is enabled):
  if (params_.loop_closure_enabled) {
    Tab tab;
    tab.title = "Loop Closure";
    tab.widgets.emplace_back(Label{gui_.lbLcLoops});
    tab.widgets.emplace_back(Label{gui_.lbLcCandidates});
    tab.widgets.emplace_back(Label{gui_.lbLcCurrent});
    tab.widgets.emplace_back(Label{gui_.lbLcLast});
    tab.widgets.emplace_back(Separator{});
    tab.widgets.emplace_back(CheckBox{
      "Highlight loop edges", viz_highlight_lc_edges_.load(),
      [this](bool checked) { viz_highlight_lc_edges_.store(checked); }});
    // Manual trigger: request an immediate full scan by the background LC thread
    // (never run analyze() from here: the engine is single-thread-per-instance,
    // so a concurrent scan would race the periodic one).
    tab.widgets.emplace_back(
      Button{"Run LC scan now", 0, "", 0, [this]() { request_loop_closure_scan(); }});
    desc.tabs.emplace_back(std::move(tab));
  }

  // View tab:
  {
    Tab tab;
    tab.title = "View";
    // Viz reference frame: which frame is the scene origin (0,0,0). "enu" (the
    // default) renders the map North-oriented via T_enu_to_map (identity, hence
    // == map, until a geo-reference is estimated).
    ComboBox frameSelectionCombo{
      "Viz reference frame", {"map", "enu"}, viz_reference_frame_.load(), [this](int index) {
        viz_reference_frame_.store(index);
      }};
    setComboFixedWidthIfSupported(frameSelectionCombo, kComboFixedWidthPx);
    tab.widgets.emplace_back(frameSelectionCombo);
    ComboBox cornerScaleCombo{
      "Corner scale",
      {"Off", "0.1 m", "0.2 m", "0.3 m", "0.5 m", "1.0 m", "2.0 m"},
      closestPresetIndex(kKeyframeCornerScalePresets, viz_keyframe_corner_scale_.load()),
      [this](int index) {
        viz_keyframe_corner_scale_.store(kKeyframeCornerScalePresets.at(index));
      }};
    setComboFixedWidthIfSupported(cornerScaleCombo, kComboFixedWidthPx);
    ComboBox sphereRadiusCombo{
      "Sphere radius",
      {"Off", "0.1 m", "0.2 m", "0.3 m", "0.5 m", "1.0 m"},
      closestPresetIndex(kKeyframeSphereRadiusPresets, viz_keyframe_sphere_radius_.load()),
      [this](int index) {
        viz_keyframe_sphere_radius_.store(kKeyframeSphereRadiusPresets.at(index));
      }};
    setComboFixedWidthIfSupported(sphereRadiusCombo, kComboFixedWidthPx);
    tab.widgets.emplace_back(Row{{
      CheckBox{
        "Show keyframes", viz_show_keyframes_.load(),
        [this](bool checked) { viz_show_keyframes_.store(checked); }},
      std::move(cornerScaleCombo),
      std::move(sphereRadiusCombo),
    }});
    ComboBox edgeRadiusCombo{
      "Edge radius",
      {"0.02 m", "0.05 m", "0.1 m", "0.2 m", "0.3 m", "0.5 m"},
      closestPresetIndex(kEdgeCylinderRadiusPresets, viz_edge_cylinder_radius_.load()),
      [this](int index) { viz_edge_cylinder_radius_.store(kEdgeCylinderRadiusPresets.at(index)); }};
    setComboFixedWidthIfSupported(edgeRadiusCombo, kComboFixedWidthPx);
    tab.widgets.emplace_back(Row{{
      CheckBox{
        "Show graph edges", viz_show_edges_.load(),
        [this](bool checked) { viz_show_edges_.store(checked); }},
      std::move(edgeRadiusCombo),
    }});
    tab.widgets.emplace_back(CheckBox{
      "Show odometry frames", viz_show_odom_frames_.load(),
      [this](bool checked) { viz_show_odom_frames_.store(checked); }});
    tab.widgets.emplace_back(CheckBox{
      "Camera follows vehicle", viz_camera_follows_vehicle_.load(),
      [this](bool checked) { viz_camera_follows_vehicle_.store(checked); }});
    tab.widgets.emplace_back(CheckBox{
      "Show ground grid", viz_show_ground_grid_.load(),
      [this](bool checked) { viz_show_ground_grid_.store(checked); }});
    desc.tabs.emplace_back(std::move(tab));
  }

  // Output tab:
  {
    Tab tab;
    tab.title = "Output";
    auto lbOutputHint =
      std::make_shared<LiveString>("Trajectory/simplemap saved at exit or when button clicked");
    tab.widgets.emplace_back(Label{lbOutputHint});
    tab.widgets.emplace_back(
      TextBox{"TUM file:", params_.save_trajectory_to_file, 13, [this](std::string f) -> bool {
                auto lck = mrpt::lockHelper(stateMutex_);
                params_.save_trajectory_to_file = std::move(f);
                return true;
              }});
    tab.widgets.emplace_back(
      Button{"Save trajectory now", 0, "", 0, [this]() { this->saveEstimatedTrajectoryToFile(); }});
    tab.widgets.emplace_back(
      TextBox{".simplemap file:", params_.save_simplemap_file, 13, [this](std::string f) -> bool {
                auto lck = mrpt::lockHelper(stateMutex_);
                params_.save_simplemap_file = std::move(f);
                return true;
              }});
    tab.widgets.emplace_back(
      Button{"Save simplemap now", 0, "", 0, [this]() { this->saveSimpleMapToFile(); }});
    desc.tabs.emplace_back(std::move(tab));
  }

  visualizer_->create_subwindow_from_description(desc).get();
}

}  // namespace mola::mapper
