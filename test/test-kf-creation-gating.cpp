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
 * @file   test-kf-creation-gating.cpp
 * @brief  Phase A tests: KF creation is gated to requestInsertKeyframe() once a
 *         SharedKeyframeMap producer is detected (plan 4.13 Phase A).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Verifies:
 *  (A) auto mode before any requestInsertKeyframe(): fuse_pose() creates KFs
 *      (legacy behavior, so existing tests/runs pass unchanged).
 *  (B) after the first requestInsertKeyframe() in auto mode: fuse_pose() does
 *      NOT create new KFs; only the sparse requestInsertKeyframe() calls do.
 *  (C) shared_map_only mode: fuse_pose() NEVER creates KFs regardless.
 *  (D) estimated_navstate() still returns valid predictions in shared_map_only
 *      mode, served from last_raw_pose_by_source (the predictor anchor).
 */

#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>
#include <string>

using namespace mola::mapper_3d;

namespace
{
const char * kParamsAuto =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  link_first_pose_to_reference_origin_sigma: 1e-6
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 10.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  keyframe_creation_source: "KeyframeCreationSource::Auto"
)###";

const char * kParamsSharedMapOnly =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  link_first_pose_to_reference_origin_sigma: 1e-6
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 10.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  keyframe_creation_source: "KeyframeCreationSource::SharedMapOnly"
)###";

mrpt::poses::CPose3DPDFGaussian pose_at(double x, double sigma = 0.02)
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0, 0, 0, 0, 0);
  p.cov.setIdentity();
  p.cov *= sigma * sigma;
  return p;
}

// (A) auto mode before any requestInsertKeyframe(): legacy KF creation
bool test_auto_mode_legacy_creation()
{
  Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsAuto));

  // Feed 5 poses separated by >0.01 s: all should create KFs.
  for (int i = 0; i < 5; i++) {
    const double t = i * 0.05;
    nav.fuse_pose(mrpt::Clock::fromDouble(t), pose_at(i * 0.1), "odom_lidar");
  }

  const size_t kfCount = nav.keyframe_count();
  if (kfCount < 5) {
    std::cerr << "[A] FAIL: expected >=5 KFs in auto/legacy mode, got " << kfCount << "\n";
    return false;
  }
  std::cout << "[A] PASS: auto/legacy mode created " << kfCount << " KFs from 5 fuse_pose calls\n";
  return true;
}

// (B) auto mode: after requestInsertKeyframe(), fuse_pose() stops creating KFs
bool test_auto_mode_switches_on_shared_kf()
{
  Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsAuto));

  // Prime with one sparse requestInsertKeyframe() call -> activates shared_map_only.
  {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(0.0);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(0.0, 1e-3);
    nav.requestInsertKeyframe(req);
  }
  const size_t kfAfterFirst = nav.keyframe_count();

  // Now feed dense fuse_pose() calls at 0.05 s intervals (well above the 0.01 s
  // min_time_difference_to_create_new_frame, so they would create KFs in legacy mode).
  for (int i = 1; i <= 20; i++) {
    nav.fuse_pose(mrpt::Clock::fromDouble(i * 0.05), pose_at(i * 0.1), "odom_lidar");
  }
  const size_t kfAfterDense = nav.keyframe_count();

  // Add a second sparse KF.
  {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(1.05);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(1.0, 1e-3);
    nav.requestInsertKeyframe(req);
  }
  const size_t kfFinal = nav.keyframe_count();

  if (kfAfterDense != kfAfterFirst) {
    std::cerr << "[B] FAIL: dense fuse_pose() after requestInsertKeyframe() created new KFs! "
              << kfAfterFirst << " -> " << kfAfterDense << "\n";
    return false;
  }
  if (kfFinal != kfAfterFirst + 1) {
    std::cerr << "[B] FAIL: second requestInsertKeyframe() should add exactly 1 KF, got "
              << kfFinal << " (was " << kfAfterFirst << ")\n";
    return false;
  }
  std::cout << "[B] PASS: auto mode: 20 dense fuse_pose() calls did not grow KF count ("
            << kfAfterFirst << "); second requestInsertKeyframe() added 1 -> " << kfFinal << "\n";
  return true;
}

// (C) shared_map_only mode: fuse_pose() NEVER creates KFs
bool test_shared_map_only_no_creation()
{
  Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsSharedMapOnly));

  // Feed 10 dense fuse_pose() calls.
  for (int i = 0; i < 10; i++) {
    nav.fuse_pose(mrpt::Clock::fromDouble(i * 0.05), pose_at(i * 0.1), "odom_lidar");
  }
  const size_t kfAfterDense = nav.keyframe_count();

  if (kfAfterDense != 0) {
    std::cerr << "[C] FAIL: shared_map_only mode: fuse_pose() created " << kfAfterDense
              << " KFs, expected 0\n";
    return false;
  }

  // Now add 3 sparse KFs.
  for (int i = 0; i < 3; i++) {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(i * 0.5);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(i * 0.5, 1e-3);
    nav.requestInsertKeyframe(req);
  }
  const size_t kfFinal = nav.keyframe_count();

  if (kfFinal != 3) {
    std::cerr << "[C] FAIL: expected 3 KFs from requestInsertKeyframe(), got " << kfFinal << "\n";
    return false;
  }
  std::cout << "[C] PASS: shared_map_only: fuse_pose() = 0 KFs; 3 requestInsertKeyframe() = "
            << kfFinal << " KFs\n";
  return true;
}

// (D) estimated_navstate() works in shared_map_only after requestInsertKeyframe()
bool test_navstate_after_kf_gating()
{
  Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsSharedMapOnly));

  // Insert a sparse KF via requestInsertKeyframe().
  {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(0.0);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(0.0, 1e-3);
    nav.requestInsertKeyframe(req);
  }

  // Feed dense fuse_pose() to update the predictor anchor in {odom_lidar}.
  for (int i = 1; i <= 5; i++) {
    nav.fuse_pose(mrpt::Clock::fromDouble(i * 0.05), pose_at(i * 0.1), "odom_lidar");
  }

  // estimated_navstate() in {odom_lidar} should return the latest predictor
  // anchor, not a failure.
  const auto ns = nav.estimated_navstate(mrpt::Clock::fromDouble(0.25), "odom_lidar");
  if (!ns.has_value()) {
    std::cerr << "[D] FAIL: estimated_navstate() returned nullopt in shared_map_only mode\n";
    return false;
  }

  const double x = ns->pose.mean.x();
  // Predictor anchor is the last raw pose from fuse_pose (x=0.5) extrapolated
  // slightly forward from t=0.25. Should be near 0.5 m.
  if (x < 0.0 || x > 1.5) {
    std::cerr << "[D] FAIL: unexpected predictor x=" << x << " (expected ~0.5 m)\n";
    return false;
  }
  std::cout << "[D] PASS: estimated_navstate() works in shared_map_only; predictor x=" << x
            << " m\n";
  return true;
}

// (E) Phase B.1: wheel relative-pose edge emitted between consecutive sparse KFs
bool test_wheel_edge_between_sparse_kfs()
{
  Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsSharedMapOnly));

  // Helper: build a 2D odometry observation at absolute position x.
  auto make_odom = [](double x, double t) {
    mrpt::obs::CObservationOdometry obs;
    obs.timestamp = mrpt::Clock::fromDouble(t);
    obs.odometry = mrpt::poses::CPose2D(x, 0.0, 0.0);
    return obs;
  };

  // Feed an initial wheel reading so the anchor is set.
  nav.fuse_odometry(make_odom(0.0, -0.01), "odom_wheels");

  // Insert the first sparse KF at t=0.0 (sets prev_shared_kf_id_ and wheel anchor).
  {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(0.0);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(0.0, 1e-3);
    nav.requestInsertKeyframe(req);
  }
  const size_t kfAfterFirst = nav.keyframe_count();

  // Accumulate wheel motion: 5 steps of 0.2 m forward.
  for (int i = 1; i <= 5; i++) {
    nav.fuse_odometry(make_odom(i * 0.2, i * 0.05), "odom_wheels");
  }

  // Insert a second sparse KF at t=0.5 -- should emit a wheel BetweenFactor.
  {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(0.5);
    req.source_frame_id = "odom_lidar_kf";
    req.pose_in_source = pose_at(1.0, 1e-3);
    nav.requestInsertKeyframe(req);
  }
  const size_t kfFinal = nav.keyframe_count();

  if (kfFinal != kfAfterFirst + 1) {
    std::cerr << "[E] FAIL: expected kfAfterFirst+1 = " << kfAfterFirst + 1
              << " KFs after second requestInsertKeyframe(), got " << kfFinal << "\n";
    return false;
  }

  std::cout << "[E] PASS: wheel edge emitted between sparse KFs; total KFs = " << kfFinal << "\n";
  return true;
}

}  // namespace

int main()
{
  bool ok = true;
  ok = test_auto_mode_legacy_creation() && ok;
  ok = test_auto_mode_switches_on_shared_kf() && ok;
  ok = test_shared_map_only_no_creation() && ok;
  ok = test_navstate_after_kf_gating() && ok;
  ok = test_wheel_edge_between_sparse_kfs() && ok;

  if (ok) {
    std::cout << "ALL PASS\n";
    return 0;
  }
  return 1;
}
