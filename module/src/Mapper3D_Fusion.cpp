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
 * @file   Mapper3D_Fusion.cpp
 * @brief  Sensor fusion: keyframe management and the native GTSAM/iSAM2 graph.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Native single-graph fusion: one gtsam::ISAM2 over ALL keyframes (the central
 * map, not a fixed-lag window), with persistent T_enu_to_map and per-source
 * T_map_to_odom_i variables. The kinematic/twist factors come from
 * mola_gtsam_factors via factor_builders.h, shared with the smoother.
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/Pose3RotationFactor.h>
#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/system/datetime.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <cmath>

#include "GtsamData.h"
#include "factor_builders.h"

namespace mola::mapper_3d
{
namespace
{
constexpr double ENU2MAP_WEAK_SIGMA = 1e4;
constexpr double INIT_ODOM_FRAME_POSE_SIGMA = 1e3;
constexpr double FIRST_POSE_WEAK_PRIOR_SIGMA = 1e6;
constexpr double PLANAR_XY_SIGMA = 1e10;
constexpr double PLANAR_Z_SIGMA = 1e-4;

// Floor for fuse_pose() input covariances. Front ends occasionally report
// pathologically tiny (but nonzero) sigmas, e.g. a relocalization seed pinned
// at ~1e-6 m. Mixed with much weaker priors/frame ties in the same linear
// system (e.g. INIT_ODOM_FRAME_POSE_SIGMA above), the resulting ~1e18 spread
// in information magnitudes was observed to throw
// gtsam::IndeterminantLinearSystemException in iSAM2's incremental Cholesky.
constexpr double MIN_POSE_SIGMA_LIN = 1e-3;  // [m]
constexpr double MIN_POSE_SIGMA_ANG = 1e-4;  // [rad]

void enforce_planar_pose(mrpt::poses::CPose3D & p)
{
  p.z(0);
  p.setYawPitchRoll(p.yaw(), .0, .0);
}
void enforce_planar_twist(mrpt::math::TTwist3D & tw)
{
  tw.vz = 0;
  tw.wx = 0;
  tw.wy = 0;
}

/// Integrates a constant body-frame twist over `dt` seconds, returning the
/// relative pose increment T_i_to_j (to be right-composed onto the anchor pose:
/// T_j = T_i (+) delta). Branches on the configured kinematic model so the
/// short-term extrapolation matches the same motion model used to build the
/// inter-keyframe factors:
///   - ConstantVelocity: full SE(3) exponential of the 6D body twist.
///   - Tricycle: planar arc from forward velocity v=vx and yaw rate w=wz (vy,
///     vz, wx, wy ignored), mirroring mola::factors::FactorTricycleKinematic's
///     integration exactly (Rz(theta), dx=R*sin(theta), dy=R*(1-cos(theta))).
mrpt::poses::CPose3D body_twist_delta(
  const Parameters & params, const mrpt::math::TTwist3D & twist, double dt)
{
  switch (params.kinematic_model) {
    case KinematicModel::Tricycle: {
      // Threshold mirrors FactorTricycleKinematic's default w_threshold_.
      constexpr double w_threshold = 1e-4;  // [rad/s]
      const double v = twist.vx;
      const double w = twist.wz;
      if (std::abs(w) < w_threshold) {
        return mrpt::poses::CPose3D(v * dt, .0, .0, .0, .0, .0);
      }
      const double R = v / w;
      const double theta = w * dt;
      const double dx = R * std::sin(theta);
      const double dy = R * (1.0 - std::cos(theta));
      // CPose3D(x, y, z, yaw, pitch, roll); the arc rotates about +z (yaw).
      return mrpt::poses::CPose3D(dx, dy, .0, theta, .0, .0);
    }
    case KinematicModel::ConstantVelocity:
    default: {
      mrpt::math::CVectorFixed<double, 6> twistDt;
      twistDt[0] = twist.vx;
      twistDt[1] = twist.vy;
      twistDt[2] = twist.vz;
      twistDt[3] = twist.wx;
      twistDt[4] = twist.wy;
      twistDt[5] = twist.wz;
      twistDt *= dt;
      return mrpt::poses::CPose3D(mrpt::poses::Lie::SE<3>::exp(twistDt));
    }
  }
}

/// Returns (max position sigma [m], max orientation sigma [deg]) from an SE(3)
/// covariance using MRPT's (x, y, z, yaw, pitch, roll) convention.
std::pair<double, double> max_pos_and_orientation_sigma(const mrpt::math::CMatrixDouble66 & cov)
{
  const double maxPosSigma = std::sqrt(std::max({cov(0, 0), cov(1, 1), cov(2, 2)}));
  const double maxOriSigmaDeg =
    mrpt::RAD2DEG(std::sqrt(std::max({cov(3, 3), cov(4, 4), cov(5, 5)})));
  return {maxPosSigma, maxOriSigmaDeg};
}
}  // namespace

// ---------------------------------------------------------------------------
// GTSAM (re)initialization
// ---------------------------------------------------------------------------
void Mapper3D::reinitialize_gtsam_locked()
{
  gtsam::ISAM2Params isam2Params;
  isam2Params.relinearizeThreshold = 0.1;
  isam2Params.relinearizeSkip = 1;
  state_.gtsam->isam2.emplace(isam2Params);

  // Always define the persistent T_enu_to_map variable, so it can be used for
  // gravity-alignment via IMU accelerometer even without GNSS.
  auto enu2map = gtsam::Pose3::Identity();
  gtsam::Matrix6 enu2map_cov = gtsam::Matrix6::Identity() * mrpt::square(ENU2MAP_WEAK_SIGMA);

  if (params_.fixed_geo_reference.has_value()) {
    state_.geo_reference = *params_.fixed_geo_reference;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(
      state_.geo_reference->T_enu_to_map, enu2map, enu2map_cov);
    state_.last_estimated_frames[REFERENCE_FRAME_ID] = state_.geo_reference->T_enu_to_map;
  }

  state_.gtsam->newValues.insert(symbol_T_enu_to_map, enu2map);
  state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);
}

// ---------------------------------------------------------------------------
// Odometry frame registry (creates the F(k) variable + weak prior)
// ---------------------------------------------------------------------------
OdometryFrameID Mapper3D::add_or_get_odom_frame_id_locked(const std::string & frame_id_name)
{
  // F(0) is special: the reference frame ("map"), not a floating odometry frame.
  if (frame_id_name == params_.reference_frame_name) {
    return REFERENCE_FRAME_ID;
  }

  ASSERT_NOT_EQUAL_(frame_id_name, params_.vehicle_frame_name);
  ASSERT_NOT_EQUAL_(frame_id_name, params_.enu_frame_name);

  if (auto it = state_.known_odom_frames.find_key(frame_id_name);
      it != state_.known_odom_frames.getDirectMap().end()) {
    return it->second;
  }

  // New odometry frame (ids start at 1; 0 is reserved for {map}/{enu}):
  const OdometryFrameID newId = state_.next_odom_frame_id++;
  state_.known_odom_frames.insert(frame_id_name, newId);

  const gtsam::Pose3 initFramePose = gtsam::Pose3::Identity();
  state_.gtsam->newValues.insert(symbol_T_map_to_odom_i_base + newId, initFramePose);
  state_.gtsam->newFactors.addPrior(
    symbol_T_map_to_odom_i_base + newId, initFramePose,
    gtsam::noiseModel::Isotropic::Sigma(6, INIT_ODOM_FRAME_POSE_SIGMA));

  return newId;
}

// ---------------------------------------------------------------------------
// Keyframe creation (with the out-of-order guard) + variable/factor seeding
// ---------------------------------------------------------------------------
KeyFrameID Mapper3D::create_or_get_keyframe_by_timestamp_locked(
  const mrpt::Clock::time_point & t, const std::optional<double> & overrideCloseEnough)
{
  const double threshold =
    overrideCloseEnough.value_or(params_.min_time_difference_to_create_new_frame);

  const auto & dm = state_.time_to_kf_id.getDirectMap();

  // Reuse an existing keyframe close enough in time?
  const auto closestPrior = find_before_after(t, true);
  for (const auto & it : {closestPrior.first, closestPrior.second}) {
    if (it == dm.end()) {
      continue;
    }
    const double dt = std::abs(mrpt::system::timeDifference(it->first, t));
    if (dt < threshold) {
      return it->second;
    }
  }

  // Out-of-order guard: a request older than the newest keyframe snaps to the
  // nearest existing keyframe instead of inserting a variable in the past
  // (iSAM2 incremental marginalization needs non-decreasing keyframe stamps).
  if (!state_.time_to_kf_id.empty()) {
    const auto newest_t = dm.rbegin()->first;
    if (t < newest_t) {
      std::optional<KeyFrameID> nearestIdx;
      double nearestDt = 0;
      for (const auto & it : {closestPrior.first, closestPrior.second}) {
        if (it == dm.end()) {
          continue;
        }
        const double dt = std::abs(mrpt::system::timeDifference(it->first, t));
        if (!nearestIdx.has_value() || dt < nearestDt) {
          nearestDt = dt;
          nearestIdx = it->second;
        }
      }
      if (nearestIdx.has_value()) {
        MRPT_LOG_THROTTLE_WARN_FMT(
          5.0,
          "[keyframe] Out-of-order measurement (%.3f s behind newest keyframe) snapped to nearest "
          "existing keyframe (%.3f s away) to keep iSAM2 variable order monotonic.",
          mrpt::system::timeDifference(t, newest_t), nearestDt);
        return *nearestIdx;
      }
    }
  }

  // Warn (but keep going) on a large gap before this new keyframe:
  if (!state_.time_to_kf_id.empty()) {
    const double gap = mrpt::system::timeDifference(dm.rbegin()->first, t);
    if (gap > params_.time_between_frames_to_warning) {
      MRPT_LOG_THROTTLE_WARN_FMT(
        5.0, "[keyframe] Large time gap of %.2f s before the new keyframe.", gap);
    }
  }

  // Create a brand-new keyframe:
  const KeyFrameID newId = state_.generate_new_kf_id();
  state_.time_to_kf_id.insert(t, newId);
  state_.keyframe_observations[newId];  // ensure an (empty) entry exists

  // Seed GTSAM variables (T/V/W) and the first-frame priors:
  const auto closestPost = find_before_after(t, false);
  initialize_new_frame(newId, closestPost);

  // Kinematic factors to the temporal neighbors:
  if (closestPost.first != dm.end()) {
    add_kinematic_factor_between(closestPost.first->second, newId);
  }
  if (closestPost.second != dm.end()) {
    add_kinematic_factor_between(newId, closestPost.second->second);
  }

  return newId;
}

void Mapper3D::initialize_new_frame(
  KeyFrameID id, const pair_nearby_frame_iterators_t & closestFrames)
{
  const auto stamp = state_.time_to_kf_id.inverse(id);
  const auto closest_idx_opt = pick_closest(closestFrames, stamp);

  gtsam::Pose3 pose = gtsam::Pose3::Identity();
  gtsam::Point3 linVelocity = gtsam::Point3::Zero();
  gtsam::Point3 angVelocity = gtsam::Point3::Zero();

  auto & newKfState = state_.last_estimated_states[id];

  if (closest_idx_opt.has_value()) {
    const auto & kfState = state_.last_estimated_states.at(*closest_idx_opt);
    pose = mrpt::gtsam_wrappers::toPose3(kfState.pose);
    linVelocity = {kfState.twist.vx, kfState.twist.vy, kfState.twist.vz};
    angVelocity = {kfState.twist.wx, kfState.twist.wy, kfState.twist.wz};
    newKfState = kfState;
    newKfState.kinematic_links_to.clear();
  } else {
    // First ever keyframe: weak priors so the system is determinate.
    state_.gtsam->newFactors.addPrior(
      T(id), pose, gtsam::noiseModel::Isotropic::Sigma(6, FIRST_POSE_WEAK_PRIOR_SIGMA));

    const auto & tw = params_.initial_twist;
    state_.gtsam->newFactors.addPrior(
      V(id), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
      gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_lin));
    state_.gtsam->newFactors.addPrior(
      W(id), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
      gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_ang));

    if (params_.link_first_pose_to_reference_origin_sigma.has_value()) {
      state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        T(id), gtsam::Pose3::Identity(),
        gtsam::noiseModel::Isotropic::Sigma(6, *params_.link_first_pose_to_reference_origin_sigma));
    }
  }

  state_.gtsam->newValues.insert(T(id), pose);
  state_.gtsam->newValues.insert(V(id), linVelocity);
  state_.gtsam->newValues.insert(W(id), angVelocity);

  if (params_.enforce_planar_motion) {
    const auto planar_z_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(
      PLANAR_Z_SIGMA, PLANAR_Z_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA,
      PLANAR_Z_SIGMA));
    state_.gtsam->newFactors.addPrior(T(id), gtsam::Pose3::Identity(), planar_z_noise);
  }
}

void Mapper3D::add_kinematic_factor_between(KeyFrameID from, KeyFrameID to)
{
  ASSERT_NOT_EQUAL_(from, to);

  auto & fromKf = state_.last_estimated_states.at(from);
  auto & toKf = state_.last_estimated_states.at(to);

  if (fromKf.kinematic_links_to.count(to) != 0 || toKf.kinematic_links_to.count(from) != 0) {
    return;  // already added
  }
  fromKf.kinematic_links_to.insert(to);
  toKf.kinematic_links_to.insert(from);

  // NOTE: deliberately NOT mrpt::Clock::toDouble(t_to) - mrpt::Clock::toDouble(t_from):
  // that naive subtraction underflows (silently wraps to ~+1.8e12) whenever one of
  // the two timestamps is before the Unix epoch, which happens in practice for
  // datasets with relative timestamps starting near zero (e.g. KITTI), combined
  // with relocalization seeding a keyframe a couple of milliseconds before the
  // first observation. mrpt::system::timeDifference() (used everywhere else in
  // this file) is immune to that, since it differences the underlying duration
  // directly instead of converting each endpoint to an absolute double first.
  const double dt = mrpt::system::timeDifference(
    state_.time_to_kf_id.inverse(from), state_.time_to_kf_id.inverse(to));

  add_kinematic_factors(state_.gtsam->newFactors, params_, from, to, dt);

  // Track topological connectivity for later loop-closure gating:
  state_.add_kf_connectivity(from, to);
}

// ---------------------------------------------------------------------------
// Sensor fusion entry points
// ---------------------------------------------------------------------------
void Mapper3D::fuse_pose(
  const mrpt::Clock::time_point & timestamp, const mrpt::poses::CPose3DPDFGaussian & pose,
  const std::string & frame_id)
{
  const ProfilerEntry tle(profiler_, "fuse_pose");
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_pose_locked(timestamp, pose, frame_id);
  }
  notify_optimizer();
}

void Mapper3D::fuse_pose_locked(
  const mrpt::Clock::time_point & timestamp, const mrpt::poses::CPose3DPDFGaussian & pose,
  const std::string & frame_id)
{
  const auto frame_id_idx = add_or_get_odom_frame_id_locked(frame_id);

  // Numerical sanity: replace zero-variance diagonal entries (common in
  // nav_msgs/Odometry with unfilled covariance) with reasonable defaults, and
  // floor pathologically tiny ones (see MIN_POSE_SIGMA_LIN/ANG above).
  auto poseSanitized = pose;
  for (int i = 0; i < 6; i++) {
    if (poseSanitized.cov(i, i) <= .0) {
      const double defaultSigma = (i < 3) ? 1.0 : 0.1;
      poseSanitized.cov(i, i) = defaultSigma * defaultSigma;
    } else {
      const double minSigma = (i < 3) ? MIN_POSE_SIGMA_LIN : MIN_POSE_SIGMA_ANG;
      poseSanitized.cov(i, i) = std::max(poseSanitized.cov(i, i), minSigma * minSigma);
    }
  }

  gtsam::Pose3 pose_out;
  gtsam::Matrix6 cov_out;
  mrpt::gtsam_wrappers::to_gtsam_se3_cov6(poseSanitized, pose_out, cov_out);

  if (frame_id_idx == REFERENCE_FRAME_ID) {
    // Reference frame ({map}): a direct prior on the keyframe. Always allowed
    // even in SharedMapOnly mode (this is a relocalization seed, not a dense
    // scan; it must land somewhere in the graph to serve as an anchor).
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp);
    state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      T(this_kf_id), pose_out, gtsam::noiseModel::Gaussian::Covariance(cov_out));
    return;
  }

  // Odometry frame: ALWAYS record the predictor anchor so estimated_navstate()
  // in {odom_i} remains fresh regardless of whether we create a KF.
  state_.last_raw_pose_by_source[frame_id_idx] = {timestamp, poseSanitized};

  if (!sensor_kf_creation_allowed()) {
    // SharedMapOnly (or Auto after first requestInsertKeyframe()): dense
    // fuse_pose() only serves the predictor. Keyframe GTSAM variables are
    // created exclusively by requestInsertKeyframe(). Returning here avoids
    // spawning a graph variable for every dense LIO scan (~10 Hz), which was
    // the dominant source of unbounded graph growth on DCC01 (~5400 KFs).
    return;
  }

  // Sensor KF creation allowed (Auto before any SharedKeyframeMap producer, or
  // SensorClock mode). Feed into the single consecutive relative-pose-edge
  // chain (plan 2.8, mirrors mola_sm_loop_closure::add_odometry_edges). Do
  // NOT add an absolute Between(F(odom_i), T(kf)) tie per pose: a single rigid
  // T_map_to_odom_i cannot fit the whole central map once odometry drifts, so
  // such ties become globally inconsistent over all keyframes and deform {map}
  // (catastrophic z/tilt). See link_into_odometry_chain_locked().
  const std::optional<double> kfOverride =
    (params_.keyframe_creation_source == KeyframeCreationSource::SensorClock)
      ? std::optional<double>(params_.sensor_clock_min_period_s)
      : std::nullopt;
  const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp, kfOverride);
  link_into_odometry_chain_locked(this_kf_id, poseSanitized, frame_id_idx);
}

void Mapper3D::link_into_odometry_chain_locked(
  KeyFrameID kf, const mrpt::poses::CPose3DPDFGaussian & absOdomPosePdf, OdometryFrameID frameIdx)
{
  const mrpt::poses::CPose3D & absOdomPose = absOdomPosePdf.mean;

  // Store the absolute odometry pose PDF (mean + covariance) that defines this
  // keyframe (first writer wins, so edge endpoints stay stable once an edge
  // using them is built). The covariance feeds the anisotropic edge noise in
  // add_odom_chain_edge_locked().
  const bool firstForThisKf = kf_odom_abs_pose_.find(kf) == kf_odom_abs_pose_.end();
  if (firstForThisKf) {
    kf_odom_abs_pose_.emplace(kf, absOdomPosePdf);
  }

  // Track the latest (newest in time) keyframe this source contributed to, for
  // the instantaneous T_map_to_odom_i report (see latest_kf_by_odom_frame_).
  if (frameIdx != REFERENCE_FRAME_ID) {
    const auto itLatest = latest_kf_by_odom_frame_.find(frameIdx);
    if (
      itLatest == latest_kf_by_odom_frame_.end() ||
      state_.time_to_kf_id.inverse(kf) >= state_.time_to_kf_id.inverse(itLatest->second)) {
      latest_kf_by_odom_frame_[frameIdx] = kf;
    }
  }

  // F(frameIdx) = T_map_to_odom_i is NOT a fusion unknown anymore: odometry is
  // fused as frame-invariant relative-pose edges (below), and T_map_to_odom_i is
  // REPORTED as the instantaneous transform T(latest_kf) (+) inv(odom_pose) in
  // optimize_and_refresh() (see the F(i) design note in agents.md). The ONE
  // exception is this single first-keyframe tie: it is not per-reading fusion
  // but a one-time, weak frame-placement / gauge anchor (via F(i)'s weak prior)
  // that keeps the first keyframe determinate when no
  // link_first_pose_to_reference_origin is configured. It is never used to
  // re-assert the source's accumulated drift (it fires once).
  if (frameIdx != REFERENCE_FRAME_ID && odom_frame_anchored_.count(frameIdx) == 0) {
    odom_frame_anchored_.insert(frameIdx);
    const double linSigma = params_.keyframe_ingestion_sigma_lin;
    const double angSigma = mrpt::DEG2RAD(params_.keyframe_ingestion_sigma_ang_deg);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6{angSigma, angSigma, angSigma, linSigma, linSigma, linSigma});
    state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      symbol_T_map_to_odom_i_base + frameIdx, T(kf), mrpt::gtsam_wrappers::toPose3(absOdomPose),
      noise);
  }

  // Link to each immediate TIME-adjacent keyframe that also has a stored
  // odometry pose (one relative edge per adjacent pair, never skipping).
  const auto t = state_.time_to_kf_id.inverse(kf);
  const auto neighbors = find_before_after(t, false);
  for (const auto & it : {neighbors.first, neighbors.second}) {
    if (it == state_.time_to_kf_id.getDirectMap().end()) {
      continue;
    }
    add_odom_chain_edge_locked(kf, it->second);
  }

  // Dead-reckon the INITIAL {map} pose of a brand-new keyframe by composing the
  // previous keyframe's current estimate with the relative odometry motion,
  // instead of the stale copy-from-neighbor done by initialize_new_frame. The
  // odometry backbone is a long relative-pose chain pinned only at a far gauge
  // anchor; if each new keyframe is seeded a full motion step behind its true
  // pose, the incremental solver has to drag it forward every step and can lag
  // or wind up in a globally-twisted configuration that still satisfies all the
  // relative edges. Seeding near the dead-reckoned pose keeps both the iSAM2
  // solve and the live cached estimate in the correct basin from the start.
  if (firstForThisKf && neighbors.first != state_.time_to_kf_id.getDirectMap().end()) {
    const KeyFrameID prev = neighbors.first->second;
    const auto itPrevOdom = kf_odom_abs_pose_.find(prev);
    if (itPrevOdom != kf_odom_abs_pose_.end()) {
      const mrpt::poses::CPose3D rel = absOdomPose - itPrevOdom->second.mean;
      std::optional<gtsam::Pose3> prevMapPose;
      if (state_.gtsam->newValues.exists(T(prev))) {
        prevMapPose = state_.gtsam->newValues.at<gtsam::Pose3>(T(prev));
      } else if (state_.gtsam->estimate.exists(T(prev))) {
        prevMapPose = state_.gtsam->estimate.at<gtsam::Pose3>(T(prev));
      } else if (const auto itSt = state_.last_estimated_states.find(prev);
                 itSt != state_.last_estimated_states.end()) {
        prevMapPose = mrpt::gtsam_wrappers::toPose3(itSt->second.pose);
      }
      if (prevMapPose.has_value()) {
        const gtsam::Pose3 init = (*prevMapPose) * mrpt::gtsam_wrappers::toPose3(rel);
        if (state_.gtsam->newValues.exists(T(kf))) {
          state_.gtsam->newValues.update(T(kf), init);
        }
        // Seed the live cached estimate too, so queries / the publisher use the
        // dead-reckoned pose for keyframes the optimizer has not solved yet.
        const auto itSt = state_.last_estimated_states.find(kf);
        if (itSt != state_.last_estimated_states.end()) {
          itSt->second.pose =
            mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(init));
        }
      }
    }
  }
}

void Mapper3D::add_odom_chain_edge_locked(KeyFrameID a, KeyFrameID b)
{
  if (a == b) {
    return;
  }
  const auto ia = kf_odom_abs_pose_.find(a);
  const auto ib = kf_odom_abs_pose_.find(b);
  if (ia == kf_odom_abs_pose_.end() || ib == kf_odom_abs_pose_.end()) {
    return;
  }
  // Order endpoints by time (earlier -> later) for a canonical edge key.
  KeyFrameID from = a;
  KeyFrameID to = b;
  if (state_.time_to_kf_id.inverse(a) > state_.time_to_kf_id.inverse(b)) {
    std::swap(from, to);
  }
  if (!odom_chain_edges_.emplace(from, to).second) {
    return;  // already linked
  }

  // Relative-pose PDF between the two keyframes: this propagates the source
  // covariances (cov_to (-) cov_from), giving an ANISOTROPIC relative-motion
  // uncertainty. Faithful port of mola_sm_loop_closure::add_odometry_edges:
  // per-DOF sigma = sqrt(diag) * uncertainty_multiplier + additive floor, with
  // NO per-DOF assumptions. Leaving the drift-prone DOFs (z, roll, pitch) as
  // soft as the data says lets the absolute IMU-gravity / GNSS factors level the
  // map, instead of a hardcoded isotropic sigma pinning them.
  const mrpt::poses::CPose3DPDFGaussian relPdf =
    kf_odom_abs_pose_.at(to) - kf_odom_abs_pose_.at(from);

  // MRPT covariance order: [x, y, z, yaw, pitch, roll].
  gtsam::Vector6 sigmasXYZYPR =
    relPdf.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();
  sigmasXYZYPR *= params_.odometry_edge_uncertainty_multiplier;
  for (int k = 0; k < 3; k++) {
    sigmasXYZYPR[k] += params_.odometry_edge_min_sigma_xyz;
    sigmasXYZYPR[3 + k] += mrpt::DEG2RAD(params_.odometry_edge_min_sigma_ang_deg);
  }

  // GTSAM Pose3 tangent-space order: (Rx=roll, Ry=pitch, Rz=yaw, tx, ty, tz).
  gtsam::Vector6 sigmas;
  sigmas << sigmasXYZYPR[5], sigmasXYZYPR[4], sigmasXYZYPR[3], sigmasXYZYPR[0], sigmasXYZYPR[1],
    sigmasXYZYPR[2];
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    T(from), T(to), mrpt::gtsam_wrappers::toPose3(relPdf.mean), noise);
  state_.add_kf_connectivity(from, to);
}

void Mapper3D::fuse_odometry(
  const mrpt::obs::CObservationOdometry & odom, const std::string & odomName)
{
  const ProfilerEntry tle(profiler_, "fuse_odometry");
  auto lck = mrpt::lockHelper(stateMutex_);

  if (!last_wheels_odometry_.has_value()) {
    // First reading: store the anchor but emit no factor (zero increment).
    last_wheels_odometry_name_ = odomName;
    last_wheels_odometry_ = odom.odometry;
    last_wheels_odometry_stamp_ = odom.timestamp;
    return;
  }

  ASSERTMSG_(
    last_wheels_odometry_name_.has_value() && *last_wheels_odometry_name_ == odomName,
    "More than one different 'odomName's received for wheels odometry!");

  // High-rate cap/merge: if this reading arrives sooner than 1/max_rate after
  // the last *kept* one, drop it WITHOUT advancing the anchor, so the next kept
  // reading fuses the full accumulated increment + accumulated covariance.
  if (params_.odometry_max_insert_rate_hz > 0 && last_wheels_odometry_stamp_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_wheels_odometry_stamp_, odom.timestamp);
    if (dt < 1.0 / params_.odometry_max_insert_rate_hz) {
      return;
    }
  }

  // Aggregation mode: do NOT spawn a keyframe (nor an absolute F(wheels) factor)
  // per sample. Share keyframes at the bounded sensor cadence and emit ONE
  // relative-pose edge Between(T(prev_kf), T(cur_kf)) per keyframe transition,
  // using the net wheel motion since the anchor (the "consecutive frame edge"
  // model, like the SharedKeyframeMap chaining). Frame-invariant, so no
  // {odom_wheels} frame variable is needed at all.
  if (params_.aggregate_high_rate_into_edges) {
    if (!sensor_kf_creation_allowed()) {
      // SharedMapOnly mode: do not create sensor KFs; wheel preintegration into
      // existing KFs belongs to Phase B. Just update the integration anchors so
      // the odometry increment accumulates correctly for the next transition.
      last_wheels_odometry_name_ = odomName;
      last_wheels_odometry_ = odom.odometry;
      last_wheels_odometry_stamp_ = odom.timestamp;
      return;
    }
    const auto kfId = create_or_get_keyframe_by_timestamp_locked(
      odom.timestamp, params_.sensor_keyframe_min_period);

    if (!wheel_chain_last_kf_.has_value()) {
      wheel_chain_last_kf_ = kfId;
      wheel_chain_anchor_odom_ = odom.odometry;
    } else if (*wheel_chain_last_kf_ != kfId) {
      const auto increment = odom.odometry - *wheel_chain_anchor_odom_;

      mrpt::obs::CActionRobotMovement2D odoAct;
      odoAct.motionModelConfiguration.modelSelection =
        mrpt::obs::CActionRobotMovement2D::mmGaussian;
      odoAct.motionModelConfiguration.gaussianModel.minStdXY = 1e-3;
      odoAct.motionModelConfiguration.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.1);
      odoAct.computeFromOdometry(increment, odoAct.motionModelConfiguration);

      mrpt::poses::CPose3DPDFGaussian relPdf;
      relPdf.copyFrom(*odoAct.poseChange);
      relPdf.cov.asEigen().diagonal().array() += 1e-4;

      gtsam::Pose3 rel;
      gtsam::Matrix6 relCov;
      mrpt::gtsam_wrappers::to_gtsam_se3_cov6(relPdf, rel, relCov);
      state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        T(*wheel_chain_last_kf_), T(kfId), rel, gtsam::noiseModel::Gaussian::Covariance(relCov));
      state_.add_kf_connectivity(*wheel_chain_last_kf_, kfId);

      wheel_chain_last_kf_ = kfId;
      wheel_chain_anchor_odom_ = odom.odometry;
    }

    last_wheels_odometry_name_ = odomName;
    last_wheels_odometry_ = odom.odometry;
    last_wheels_odometry_stamp_ = odom.timestamp;

    notify_optimizer();
    return;
  }

  const mrpt::poses::CPose2D lastOdom = *last_wheels_odometry_;

  // Probabilistic motion model for the increment uncertainty:
  mrpt::obs::CActionRobotMovement2D odoAct;
  odoAct.motionModelConfiguration.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  odoAct.motionModelConfiguration.gaussianModel.minStdXY = 1e-3;
  odoAct.motionModelConfiguration.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.1);

  const auto odometryIncrement = odom.odometry - lastOdom;
  odoAct.computeFromOdometry(odometryIncrement, odoAct.motionModelConfiguration);

  mrpt::poses::CPose3DPDFGaussian newOdomPosePdf;
  newOdomPosePdf.copyFrom(*odoAct.poseChange);
  newOdomPosePdf.cov.asEigen().diagonal().array() += 1e-4;
  // Back to the global "odom" frame for fusion in that frame:
  newOdomPosePdf.changeCoordinatesReference(mrpt::poses::CPose3D(lastOdom));

  last_wheels_odometry_name_ = odomName;
  last_wheels_odometry_ = odom.odometry;
  last_wheels_odometry_stamp_ = odom.timestamp;

  fuse_pose_locked(odom.timestamp, newOdomPosePdf, odomName);

  notify_optimizer();
}

void Mapper3D::fuse_twist(
  const mrpt::Clock::time_point & timestamp, const mrpt::math::TTwist3D & twist,
  const mrpt::math::CMatrixDouble66 & twistCov)
{
  auto lck = mrpt::lockHelper(stateMutex_);

  const gtsam::Vector3 v = {twist.vx, twist.vy, twist.vz};
  const gtsam::Vector3 w = {twist.wx, twist.wy, twist.wz};
  const gtsam::Matrix3 vCov = twistCov.asEigen().block<3, 3>(0, 0);
  const gtsam::Matrix3 wCov = twistCov.asEigen().block<3, 3>(3, 3);

  const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp);
  add_twist_priors(state_.gtsam->newFactors, this_kf_id, v, vCov, w, wCov);

  notify_optimizer();
}

void Mapper3D::fuse_imu(const mrpt::obs::CObservationIMU & imu)
{
  const ProfilerEntry tle(profiler_, "fuse_imu");
  auto lck = mrpt::lockHelper(stateMutex_);

  // Path A: max-rate summarization (preferred for real high-rate IMUs). Buffer
  // every sample and insert at most imu_max_insert_rate_hz SUMMARIZED
  // observations/second: averaged accelerometer (less-noisy gravity/leveling),
  // averaged angular velocity, latest absolute orientation. This bounds BOTH the
  // factor rate AND the IMU-driven keyframe creation rate (the keyframe-reuse
  // window becomes 1/rate), without a fixed decimation ratio that would just
  // throw the in-between information away.
  if (params_.imu_max_insert_rate_hz > 0) {
    accumulate_imu_sample_locked(imu);

    const double period = 1.0 / params_.imu_max_insert_rate_hz;
    if (last_imu_summary_stamp_.has_value()) {
      const double dt = mrpt::system::timeDifference(*last_imu_summary_stamp_, imu.timestamp);
      if (dt < period) {
        return;  // keep accumulating; a summary is inserted once 1/rate elapses
      }
    }
    mrpt::obs::CObservationIMU summarized;
    if (!build_summarized_imu_locked(summarized)) {
      return;
    }
    last_imu_summary_stamp_ = summarized.timestamp;
    apply_imu_observation_locked(summarized, period);
    notify_optimizer();
    return;
  }

  // Path B: no rate cap (imu_max_insert_rate_hz == 0): insert every reading.
  // In aggregation mode, attach to the shared bounded-rate keyframe clock (the
  // same one wheel odometry uses), so IMU does not spawn its own dense
  // keyframes either.
  const double imuKfTolerance = params_.aggregate_high_rate_into_edges
                                  ? params_.sensor_keyframe_min_period
                                  : params_.imu_nearby_keyframe_stamp_tolerance;
  apply_imu_observation_locked(imu, imuKfTolerance);
  notify_optimizer();
}

void Mapper3D::accumulate_imu_sample_locked(const mrpt::obs::CObservationIMU & imu)
{
  if (imu.has(mrpt::obs::IMU_X_ACC)) {
    imu_accum_.acc_sum[0] += imu.get(mrpt::obs::IMU_X_ACC);
    imu_accum_.acc_sum[1] += imu.get(mrpt::obs::IMU_Y_ACC);
    imu_accum_.acc_sum[2] += imu.get(mrpt::obs::IMU_Z_ACC);
    imu_accum_.n_acc++;
  }
  if (imu.has(mrpt::obs::IMU_WX)) {
    imu_accum_.gyro_sum[0] += imu.get(mrpt::obs::IMU_WX);
    imu_accum_.gyro_sum[1] += imu.get(mrpt::obs::IMU_WY);
    imu_accum_.gyro_sum[2] += imu.get(mrpt::obs::IMU_WZ);
    imu_accum_.n_gyro++;
  }
  if (imu.has(mrpt::obs::IMU_ORI_QUAT_W)) {
    // Orientation can't be linearly averaged; keep the latest (window is short
    // at >= a few Hz). It is the absolute-attitude observation for this summary.
    imu_accum_.quat_wxyz[0] = imu.get(mrpt::obs::IMU_ORI_QUAT_W);
    imu_accum_.quat_wxyz[1] = imu.get(mrpt::obs::IMU_ORI_QUAT_X);
    imu_accum_.quat_wxyz[2] = imu.get(mrpt::obs::IMU_ORI_QUAT_Y);
    imu_accum_.quat_wxyz[3] = imu.get(mrpt::obs::IMU_ORI_QUAT_Z);
    imu_accum_.has_quat = true;
  }
  imu_accum_.sensor_pose = imu.sensorPose;
  imu_accum_.last_stamp = imu.timestamp;
}

bool Mapper3D::build_summarized_imu_locked(mrpt::obs::CObservationIMU & out)
{
  if (imu_accum_.empty()) {
    return false;
  }
  out.sensorLabel = "imu_summary";
  out.timestamp = imu_accum_.last_stamp;
  out.sensorPose = imu_accum_.sensor_pose;
  if (imu_accum_.n_acc > 0) {
    const double inv = 1.0 / static_cast<double>(imu_accum_.n_acc);
    out.set(mrpt::obs::IMU_X_ACC, imu_accum_.acc_sum[0] * inv);
    out.set(mrpt::obs::IMU_Y_ACC, imu_accum_.acc_sum[1] * inv);
    out.set(mrpt::obs::IMU_Z_ACC, imu_accum_.acc_sum[2] * inv);
  }
  if (imu_accum_.n_gyro > 0) {
    const double inv = 1.0 / static_cast<double>(imu_accum_.n_gyro);
    out.set(mrpt::obs::IMU_WX, imu_accum_.gyro_sum[0] * inv);
    out.set(mrpt::obs::IMU_WY, imu_accum_.gyro_sum[1] * inv);
    out.set(mrpt::obs::IMU_WZ, imu_accum_.gyro_sum[2] * inv);
  }
  if (imu_accum_.has_quat) {
    out.set(mrpt::obs::IMU_ORI_QUAT_W, imu_accum_.quat_wxyz[0]);
    out.set(mrpt::obs::IMU_ORI_QUAT_X, imu_accum_.quat_wxyz[1]);
    out.set(mrpt::obs::IMU_ORI_QUAT_Y, imu_accum_.quat_wxyz[2]);
    out.set(mrpt::obs::IMU_ORI_QUAT_Z, imu_accum_.quat_wxyz[3]);
  }
  imu_accum_.clear();
  return true;
}

void Mapper3D::apply_imu_observation_locked(
  const mrpt::obs::CObservationIMU & imu, double keyframe_reuse_tolerance)
{
  KeyFrameID this_kf_id = 0;
  if (sensor_kf_creation_allowed()) {
    this_kf_id =
      create_or_get_keyframe_by_timestamp_locked(imu.timestamp, keyframe_reuse_tolerance);
  } else {
    // SharedMapOnly mode: snap IMU factors to the nearest existing KF (created
    // by requestInsertKeyframe()). If no KF exists yet, skip this reading.
    const auto nearestOpt = find_nearest_kf_locked(imu.timestamp);
    if (!nearestOpt.has_value()) {
      return;
    }
    this_kf_id = *nearestOpt;
  }

  const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

  bool addedFactor = false;

  // (1) Absolute attitude / azimuth observation (IMU orientation quaternion):
  if (imu.has(mrpt::obs::IMU_ORI_QUAT_W)) {
    mrpt::math::CQuaternionDouble q;
    q.w(imu.get(mrpt::obs::IMU_ORI_QUAT_W));
    q.x(imu.get(mrpt::obs::IMU_ORI_QUAT_X));
    q.y(imu.get(mrpt::obs::IMU_ORI_QUAT_Y));
    q.z(imu.get(mrpt::obs::IMU_ORI_QUAT_Z));
    if (std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())) {
      MRPT_LOG_THROTTLE_WARN(5.0, "Ignoring IMU orientation quaternion with NaN components");
    } else if (std::abs(q.norm() - 1.0) > 0.02) {
      MRPT_LOG_THROTTLE_WARN(5.0, "Ignoring non-normalized IMU orientation quaternion");
    } else {
      auto measuredRotation = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
      // ENU has yaw=0 => East; correct to azimuth wrt true North:
      measuredRotation =
        gtsam::Rot3::Rz(mrpt::DEG2RAD(90.0 + params_.imu_attitude_azimuth_offset_deg)) *
        measuredRotation;
      auto rotationNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params_.imu_attitude_sigma_deg));
      state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationFactor>(
        symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredRotation, rotationNoise);
      addedFactor = true;
    }
  }

  // (2) Gravity-aligned acceleration observation (accelerometer leveling):
  if (imu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0) {
    const gtsam::Vector3 measuredGravity = {
      imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC), imu.get(mrpt::obs::IMU_Z_ACC)};
    // Accept either m/s^2 (~9.8) or normalized (~1.0) accelerometer scales:
    if (
      std::abs(measuredGravity.norm() - 9.8) < 2.0 ||
      std::abs(measuredGravity.norm() - 1.0) < 0.2) {
      const gtsam::Vector3 measuredGravityNormalized = measuredGravity.normalized();
      auto accNoise = gtsam::noiseModel::Isotropic::Sigma(
        3, mrpt::DEG2RAD(params_.imu_normalized_gravity_alignment_sigma));
      state_.gtsam->newFactors.emplace_shared<mola::factors::MeasuredGravityFactor>(
        symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredGravityNormalized, accNoise);
      addedFactor = true;
    }
  }

  // (3) Optional gyroscope body-frame angular-velocity prior:
  if (params_.imu_angular_velocity_sigma_deg > 0 && imu.has(mrpt::obs::IMU_WX)) {
    const gtsam::Vector3 wSensor = {
      imu.get(mrpt::obs::IMU_WX), imu.get(mrpt::obs::IMU_WY), imu.get(mrpt::obs::IMU_WZ)};
    // Rotate from sensor frame to vehicle/body frame:
    const auto Rsv = sensorOnVehicle.rotation().matrix();
    const gtsam::Vector3 wBody = Rsv * wSensor;
    auto wNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params_.imu_angular_velocity_sigma_deg));
    state_.gtsam->newFactors.addPrior(W(this_kf_id), wBody, wNoise);
    addedFactor = true;
  }

  if (addedFactor) {
    imu_factors_inserted_++;
  }
  MRPT_LOG_THROTTLE_DEBUG_FMT(
    5.0, "[fuse_imu] kf=%zu attitude/gravity/gyro factors so far=%zu",
    static_cast<size_t>(this_kf_id), imu_factors_inserted_);
}

void Mapper3D::fuse_gnss(const mrpt::obs::CObservationGPS & gps)
{
  const ProfilerEntry tle(profiler_, "fuse_gnss");
  auto lck = mrpt::lockHelper(stateMutex_);

  // Diagnostic: dump EVERY raw GNSS reading (and why it is accepted/rejected),
  // un-throttled, when MOLA_MAPPER3D_TRACE_GPS is set. Lets us inspect the data
  // quality (height constancy + per-fix ENU covariance) end to end.
  static const bool traceGps = (::getenv("MOLA_MAPPER3D_TRACE_GPS") != nullptr);
  gnss_readings_seen_++;
  if (traceGps) {
    std::string lla = "(no GGA)";
    int fixq = -1;
    if (gps.has_GGA_datum()) {
      const auto & g = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
      const auto c = g.getAsStruct<mrpt::topography::TGeodeticCoords>();
      fixq = static_cast<int>(g.fields.fix_quality);
      lla = mrpt::format(
        "lat=%.8f lon=%.8f h=%.3f fix=%d", c.lat.decimal_value, c.lon.decimal_value, c.height,
        fixq);
    }
    std::string cov = "(no ENU cov)";
    if (gps.covariance_enu.has_value()) {
      const auto & C = *gps.covariance_enu;
      cov = mrpt::format(
        "sigma_enu=(%.2f, %.2f, %.2f) m", std::sqrt(C(0, 0)), std::sqrt(C(1, 1)),
        std::sqrt(C(2, 2)));
    }
    MRPT_LOG_INFO_FMT(
      "[GPS-TRACE] #%zu t=%.3f %s %s", static_cast<size_t>(gnss_readings_seen_),
      mrpt::Clock::toDouble(gps.timestamp), lla.c_str(), cov.c_str());
  }

  if (!gps.has_GGA_datum()) {
    MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading without GGA data.");
    return;
  }
  const auto & gga = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
  if (gga.fields.fix_quality == 0) {
    MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading; GGA has no valid fix_quality.");
    return;
  }

  const auto geoCoords = gga.getAsStruct<mrpt::topography::TGeodeticCoords>();

  // Determine the ENU reference (fixed, already-estimated, or tentative):
  std::optional<mrpt::topography::TGeodeticCoords> refGeoCoords;
  if (state_.geo_reference.has_value()) {
    refGeoCoords = state_.geo_reference->geo_coord;
  } else if (params_.estimate_geo_reference) {
    if (!state_.tentative_geo_coord_reference.has_value()) {
      state_.tentative_geo_coord_reference = geoCoords;
      MRPT_LOG_INFO_FMT(
        "[fuse_gnss] Set tentative ENU origin from first GNSS fix: lat=%.8f lon=%.8f h=%.2f "
        "(geo-ref will be estimated live as more fixes arrive).",
        geoCoords.lat.decimal_value, geoCoords.lon.decimal_value, geoCoords.height);
    }
    refGeoCoords = state_.tentative_geo_coord_reference;
  }

  if (!refGeoCoords.has_value()) {
    MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading; no fixed or tentative geo-reference.");
    return;
  }
  if (!gps.covariance_enu.has_value()) {
    MRPT_LOG_THROTTLE_WARN(5.0, "Discarding GNSS reading without ENU covariance.");
    return;
  }

  mrpt::math::TPoint3D ENU_point;
  mrpt::topography::geodeticToENU_WGS84(geoCoords, ENU_point, *refGeoCoords);

  KeyFrameID this_kf_id = 0;
  if (sensor_kf_creation_allowed()) {
    this_kf_id = create_or_get_keyframe_by_timestamp_locked(
      gps.timestamp, params_.gnss_nearby_keyframe_stamp_tolerance);
  } else {
    // SharedMapOnly mode: snap GNSS factor to nearest existing KF. If none
    // exists yet, skip (no point attaching a GNSS factor to thin air).
    const auto nearestOpt = find_nearest_kf_locked(gps.timestamp);
    if (!nearestOpt.has_value()) {
      MRPT_LOG_THROTTLE_DEBUG(
        2.0, "[fuse_gnss] No existing keyframe to attach GNSS factor to; skipping.");
      return;
    }
    this_kf_id = *nearestOpt;
  }

  const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPoint3(gps.sensorPose.translation());
  const auto observedEnu = mrpt::gtsam_wrappers::toPoint3(ENU_point);
  const auto enuNoise = gtsam::noiseModel::Gaussian::Covariance(gps.covariance_enu->asEigen());

  gtsam::SharedNoiseModel enuNoiseModel;
  if (params_.gnss_huber_threshold > 0) {
    enuNoiseModel = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.gnss_huber_threshold), enuNoise);
  } else {
    enuNoiseModel = enuNoise;
  }

  state_.gtsam->newFactors.emplace_shared<mola::factors::FactorGnssMapEnu>(
    symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, observedEnu, enuNoiseModel);
  gnss_factors_inserted_++;

  if (traceGps) {
    MRPT_LOG_INFO_FMT(
      "[GPS-TRACE] #%zu ACCEPTED -> kf=%zu ENU=(%.2f, %.2f, %.2f) sigma_enu=(%.2f, %.2f, %.2f) "
      "factors=%zu",
      static_cast<size_t>(gnss_readings_seen_), static_cast<size_t>(this_kf_id), ENU_point.x,
      ENU_point.y, ENU_point.z, std::sqrt((*gps.covariance_enu)(0, 0)),
      std::sqrt((*gps.covariance_enu)(1, 1)), std::sqrt((*gps.covariance_enu)(2, 2)),
      gnss_factors_inserted_);
  }
  MRPT_LOG_THROTTLE_DEBUG_FMT(
    2.0, "[fuse_gnss] kf=%zu ENU=(%.2f, %.2f, %.2f) fix_quality=%d factors=%zu",
    static_cast<size_t>(this_kf_id), ENU_point.x, ENU_point.y, ENU_point.z,
    static_cast<int>(gga.fields.fix_quality), gnss_factors_inserted_);

  notify_optimizer();
}

// ---------------------------------------------------------------------------
// Optimization + estimate extraction
// ---------------------------------------------------------------------------
// Three-phase, self-locking solve so the heavy GTSAM work does NOT hold
// stateMutex_ (which would block estimated_navstate() / the high-rate
// publisher / the ingest path for the full, growing solve time):
//   (A) brief stateMutex_: move out pending factors/values + snapshot which
//       keyframes/frames to refresh.
//   (B) no stateMutex_ (solve_mutex_ held): iSAM2 update + calculateEstimate +
//       marginals into local temporaries. Only this code path touches iSAM2.
//   (C) brief stateMutex_: commit the refreshed poses/twists/covariances,
//       frame transforms and (convergence-gated) geo-reference into the caches.
void Mapper3D::optimize_and_refresh()
{
  const ProfilerEntry tle(profiler_, "optimize_and_refresh");
  std::scoped_lock solveLock(solve_mutex_);

  auto & gd = *state_.gtsam;
  ASSERT_(gd.isam2.has_value());

  // ---- Phase A: grab pending work + snapshot (brief lock) ----
  gtsam::NonlinearFactorGraph localFactors;
  gtsam::Values localValues;
  // Holds the new estimate during Phase B; committed to gd.estimate under
  // stateMutex_ in Phase C so that concurrent readers of gd.estimate (e.g.
  // link_into_odometry_chain_locked) never see a partially-replaced Values.
  gtsam::Values localEstimate;
  std::vector<KeyFrameID> snapshotKfIds;
  std::vector<OdometryFrameID> snapshotFrameIds;
  // Per odom source: its latest keyframe + that keyframe's reported odom pose,
  // for the INSTANTANEOUS T_map_to_odom_i = T(latest_kf) (+) inv(odom_pose).
  std::map<OdometryFrameID, std::pair<KeyFrameID, mrpt::poses::CPose3D>> frameLatestOdom;
  KeyFrameID latestKfId = 0;
  bool haveKfs = false;
  bool estimateGeoref = false;
  bool fixedGeoref = false;
  std::optional<mrpt::topography::TGeodeticCoords> tentativeGeo;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    if (gd.newFactors.empty() && gd.newValues.empty()) {
      return;  // nothing pending
    }
    localFactors = gd.newFactors;
    gd.newFactors.resize(0);
    localValues = gd.newValues;
    gd.newValues.clear();

    snapshotKfIds.reserve(state_.last_estimated_states.size());
    for (const auto & [id, kf] : state_.last_estimated_states) {
      (void)kf;
      snapshotKfIds.push_back(id);
    }
    for (const auto & [name, fid] : state_.known_odom_frames.getDirectMap()) {
      (void)name;
      snapshotFrameIds.push_back(fid);
      const auto itLatest = latest_kf_by_odom_frame_.find(fid);
      if (itLatest != latest_kf_by_odom_frame_.end()) {
        const auto itOdom = kf_odom_abs_pose_.find(itLatest->second);
        if (itOdom != kf_odom_abs_pose_.end()) {
          frameLatestOdom.emplace(fid, std::make_pair(itLatest->second, itOdom->second.mean));
        }
      }
    }
    haveKfs = !state_.time_to_kf_id.empty();
    if (haveKfs) {
      latestKfId = state_.last_kf_id();
    }
    estimateGeoref = params_.estimate_geo_reference;
    fixedGeoref = params_.fixed_geo_reference.has_value();
    tentativeGeo = state_.tentative_geo_coord_reference;
  }

  // ---- Phase B: heavy solve, NO stateMutex_ (solve_mutex_ held) ----
  try {
    gd.isam2->update(localFactors, localValues);
    for (unsigned int i = 1; i < params_.additional_isam2_update_steps; ++i) {
      gd.isam2->update();
    }
    localEstimate = gd.isam2->calculateEstimate();
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM(
      "[optimize] iSAM2 update/estimate failed (graph may be underconstrained). "
      "Discarding this batch. Exception:\n"
      << e.what());
    return;
  }


  struct TmpKf
  {
    mrpt::poses::CPose3D pose;
    mrpt::math::TTwist3D twist;
  };
  std::map<KeyFrameID, TmpKf> tmpStates;
  std::optional<mrpt::math::CMatrixDouble66> latestPoseCov;
  std::optional<mrpt::math::CMatrixDouble66> latestTwistCov;
  std::map<OdometryFrameID, mrpt::poses::CPose3DPDFGaussian> tmpFrames;
  std::optional<mrpt::poses::CPose3DPDFGaussian> tmpEnu;
  std::optional<mola::Georeferencing> tmpGeoRef;

  try {
    for (const KeyFrameID id : snapshotKfIds) {
      if (!localEstimate.exists(T(id))) {
        continue;
      }
      TmpKf t;
      t.pose =
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(localEstimate.at<gtsam::Pose3>(T(id))));
      const auto linV = localEstimate.at<gtsam::Vector3>(V(id));
      const auto angV = localEstimate.at<gtsam::Vector3>(W(id));
      t.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};
      if (params_.enforce_planar_motion) {
        enforce_planar_pose(t.pose);
        enforce_planar_twist(t.twist);
      }
      static const bool traceVW = (::getenv("MOLA_MAPPER3D_TRACE_VW") != nullptr);
      if (traceVW && (angV.norm() > 5.0 || linV.norm() < 0.5)) {
        // dt to the time-adjacent previous keyframe (the kinematic-factor dt):
        double dtPrev = -1.0;
        const auto & m = state_.time_to_kf_id.getDirectMap();
        if (auto itT = m.find(state_.time_to_kf_id.inverse(id)); itT != m.end() && itT != m.begin()) {
          dtPrev = mrpt::system::timeDifference(std::prev(itT)->first, itT->first);
        }
        MRPT_LOG_WARN_FMT(
          "[VW-TRACE] kf=%zu V=(%.2f,%.2f,%.2f) |V|=%.2f W=(%.3f,%.3f,%.3f) |W|=%.2f dt_prev=%.4f",
          static_cast<size_t>(id), linV.x(), linV.y(), linV.z(), linV.norm(), angV.x(), angV.y(),
          angV.z(), angV.norm(), dtPrev);
      }
      tmpStates.emplace(id, t);
    }

    // Marginal covariance ONLY for the latest keyframe (the predictor's anchor
    // in steady state): cheap, and keeps the query path iSAM2-free.
    if (haveKfs && localEstimate.exists(T(latestKfId))) {
      latestPoseCov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(
        gtsam::Matrix6(gd.isam2->marginalCovariance(T(latestKfId))));
      mrpt::math::CMatrixDouble66 twCov;
      twCov.setZero();
      twCov.asEigen().block<3, 3>(0, 0) = gd.isam2->marginalCovariance(V(latestKfId));
      twCov.asEigen().block<3, 3>(3, 3) = gd.isam2->marginalCovariance(W(latestKfId));
      latestTwistCov = twCov;
    }

    // T_map_to_odom_i is the INSTANTANEOUS transform that places source i's odom
    // frame into {map}, recovered from its latest keyframe:
    //   T_map_to_odom_i = T(latest_kf) (+) inverse(odom_pose_i(latest_kf)).
    // The graph variable F(i) is no longer the fusion mechanism (odometry is
    // fused as frame-invariant relative edges, see link_into_odometry_chain),
    // so it is not read here; this live value correctly drifts over time for a
    // drifting source instead of being forced to a single rigid transform.
    for (const OdometryFrameID fid : snapshotFrameIds) {
      const auto itLatest = frameLatestOdom.find(fid);
      if (itLatest == frameLatestOdom.end()) {
        continue;
      }
      const KeyFrameID kf = itLatest->second.first;
      if (!localEstimate.exists(T(kf))) {
        continue;
      }
      const mrpt::poses::CPose3D mapPose(
        mrpt::gtsam_wrappers::toTPose3D(localEstimate.at<gtsam::Pose3>(T(kf))));
      const mrpt::poses::CPose3D & odomPose = itLatest->second.second;
      mrpt::poses::CPose3DPDFGaussian pdf;
      pdf.mean = mapPose + (mrpt::poses::CPose3D() - odomPose);  // mapPose (+) inv(odomPose)
      // Use the keyframe's marginal as a representative uncertainty (the frame
      // transform is as well-known as the keyframe it is anchored on).
      if (kf == latestKfId && latestPoseCov.has_value()) {
        pdf.cov = *latestPoseCov;
      } else {
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(0.1);
      }
      tmpFrames.emplace(fid, pdf);
    }

    if (estimateGeoref) {
      const auto Te = localEstimate.at<gtsam::Pose3>(symbol_T_enu_to_map);
      const auto Tecov = gd.isam2->marginalCovariance(symbol_T_enu_to_map);
      mrpt::poses::CPose3DPDFGaussian pdf;
      pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(Te));
      pdf.cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(Tecov);
      tmpEnu = pdf;

      // Convergence-gated geo_reference: BOTH the T_enu_to_map estimate AND the
      // latest keyframe's pose in {map} must clear the configured sigma
      // thresholds (mirrors mola_state_estimation_smoother).
      if (tentativeGeo.has_value() && !fixedGeoref && haveKfs && latestPoseCov.has_value()) {
        const auto [enuPosSigma, enuOriSigmaDeg] = max_pos_and_orientation_sigma(pdf.cov);
        const auto [kfPosSigma, kfOriSigmaDeg] = max_pos_and_orientation_sigma(*latestPoseCov);
        // Geo-ref convergence gates on the geo-reference TRANSFORM quality
        // (T_enu_to_map position + orientation) plus the keyframe ORIENTATION
        // (leveling/attitude well-determined). It deliberately does NOT gate on
        // the latest keyframe's ABSOLUTE position sigma: unlike the smoother's
        // sliding window, the central map pins a far-away gauge anchor, so that
        // absolute sigma legitimately grows with distance from the anchor (it is
        // GNSS-floored, a few meters) -- it measures the expected drift of the
        // vehicle/odometry frame wrt ENU, not whether the geo-reference itself is
        // trustworthy. Gating on it would prevent convergence from ever latching
        // on long trajectories.
        const bool converged =
          enuPosSigma <= params_.convergence_max_position_sigma &&
          std::max(enuOriSigmaDeg, kfOriSigmaDeg) <= params_.convergence_max_orientation_sigma_deg;
        MRPT_LOG_THROTTLE_DEBUG_FMT(
          2.0,
          "[geo-ref] convergence check: enu(pos=%.3f m, ori=%.3f deg) "
          "kf(pos=%.3f m [drift, not gated], ori=%.3f deg) thresh(pos=%.2f m, ori=%.2f deg) -> %s",
          enuPosSigma, enuOriSigmaDeg, kfPosSigma, kfOriSigmaDeg,
          params_.convergence_max_position_sigma, params_.convergence_max_orientation_sigma_deg,
          converged ? "CONVERGED" : "not yet");
        if (converged) {
          mola::Georeferencing gr;
          gr.geo_coord = *tentativeGeo;
          gr.T_enu_to_map = pdf;
          tmpGeoRef = gr;
        }
      }
    }
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("[optimize] estimate extraction failed:\n" << e.what());
    return;
  }

  // ---- Phase C: commit caches (brief lock) ----
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    // Commit the new estimate atomically under stateMutex_ so that concurrent
    // readers of gd.estimate (link_into_odometry_chain_locked, etc.) never
    // observe a partially-replaced Values object.
    gd.estimate = localEstimate;
    for (const auto & [id, t] : tmpStates) {
      const auto it = state_.last_estimated_states.find(id);
      if (it == state_.last_estimated_states.end()) {
        continue;  // keyframe deleted meanwhile
      }
      it->second.pose = t.pose;
      it->second.twist = t.twist;
      it->second.pose_cov.reset();
      it->second.twist_cov.reset();
    }
    if (haveKfs) {
      const auto it = state_.last_estimated_states.find(latestKfId);
      if (it != state_.last_estimated_states.end()) {
        it->second.pose_cov = latestPoseCov;
        it->second.twist_cov = latestTwistCov;
      }
    }
    std::string driftTrace;
    const auto & idToName = state_.known_odom_frames.getInverseMap();
    for (const auto & [fid, pdf] : tmpFrames) {
      state_.last_estimated_frames[fid] = pdf;
      const auto itName = idToName.find(fid);
      const std::string nm = (itName != idToName.end()) ? itName->second : std::to_string(fid);
      const double cosAngle =
        std::clamp((pdf.mean.getRotationMatrix().trace() - 1.0) * 0.5, -1.0, 1.0);
      driftTrace += mrpt::format(
        " %s=%.2fm/%.1fdeg", nm.c_str(), pdf.mean.translation().norm(),
        mrpt::RAD2DEG(std::acos(cosAngle)));
    }
    if (!driftTrace.empty()) {
      MRPT_LOG_THROTTLE_DEBUG_FMT(
        5.0, "[frames] T_map_to_odom_i drift (trans/rot):%s", driftTrace.c_str());
    }
    if (tmpEnu.has_value()) {
      state_.last_estimated_frames[REFERENCE_FRAME_ID] = *tmpEnu;
    }
    if (tmpGeoRef.has_value()) {
      if (!georef_converged_announced_) {
        georef_converged_announced_ = true;
        const auto [posSigma, oriSigmaDeg] =
          max_pos_and_orientation_sigma(tmpGeoRef->T_enu_to_map.cov);
        const auto & m = tmpGeoRef->T_enu_to_map.mean;
        MRPT_LOG_INFO_FMT(
          "[geo-ref] CONVERGED: T_enu_to_map=(%.2f, %.2f, %.2f, yaw=%.2f deg) "
          "sigma_pos=%.3f m sigma_ori=%.3f deg after %zu GNSS factors.",
          m.x(), m.y(), m.z(), mrpt::RAD2DEG(m.yaw()), posSigma, oriSigmaDeg,
          gnss_factors_inserted_);
      }
      state_.geo_reference = tmpGeoRef;
    }
  }
}

NavState Mapper3D::get_latest_state_and_covariance(KeyFrameID idx) const
{
  const auto & frame = state_.last_estimated_states.at(idx);

  NavState ns;
  ns.pose.mean = frame.pose;
  ns.twist = frame.twist;

  // Covariances come from the cache (filled by optimize_and_refresh for the
  // latest keyframe), so this query path never touches iSAM2 and can run
  // concurrently with the background optimizer. Prefer this keyframe's own
  // cached covariance; fall back to the latest keyframe's; last resort a large
  // default.
  std::optional<mrpt::math::CMatrixDouble66> poseCov = frame.pose_cov;
  std::optional<mrpt::math::CMatrixDouble66> twistCov = frame.twist_cov;
  if (!poseCov.has_value() && !state_.time_to_kf_id.empty()) {
    const auto it = state_.last_estimated_states.find(state_.last_kf_id());
    if (it != state_.last_estimated_states.end()) {
      poseCov = it->second.pose_cov;
      twistCov = it->second.twist_cov;
    }
  }

  mrpt::math::CMatrixDouble66 pc;
  if (poseCov.has_value()) {
    pc = *poseCov;
  } else {
    pc.setIdentity();
    pc *= 1e6;
  }
  ns.pose.cov_inv = pc.inverse_LLt();

  mrpt::math::CMatrixDouble66 tc;
  if (twistCov.has_value()) {
    tc = *twistCov;
  } else {
    tc.setIdentity();
    tc *= 1e6;
  }
  ns.twist_inv_cov = tc.inverse_LLt();

  return ns;
}

std::optional<NavState> Mapper3D::estimated_navstate(
  const mrpt::Clock::time_point & timestamp, const std::string & frame_id)
{
  const ProfilerEntry tle(profiler_, "estimated_navstate");
  // 1) Ensure cached estimates are up to date. With the background optimizer
  // thread, the caches are refreshed off the query path (so this returns
  // quickly from the last committed solve); otherwise flush synchronously here
  // (deterministic single-threaded path, e.g. unit tests). NOTE: called with NO
  // lock held, since optimize_and_refresh() does its own (brief) locking.
  if (!params_.enable_optimizer_thread) {
    optimize_and_refresh();
  }

  auto lck = mrpt::lockHelper(stateMutex_);

  if (state_.last_estimated_states.empty()) {
    return {};
  }

  // 2) Find the closest keyframe to the requested time.
  std::optional<double> closestFrameDt;
  double closestFrameDtSigned = 0;
  std::optional<KeyFrameID> closestFrameIdx;

  const auto closestPrior = find_before_after(timestamp, true);
  for (const auto & it : {closestPrior.first, closestPrior.second}) {
    if (it == state_.time_to_kf_id.getDirectMap().end()) {
      continue;
    }
    const double dt = mrpt::system::timeDifference(it->first, timestamp);
    const double dtAbs = std::abs(dt);
    if (!closestFrameDt.has_value() || dtAbs < *closestFrameDt) {
      closestFrameDt = dtAbs;
      closestFrameIdx = it->second;
      closestFrameDtSigned = dt;
    }
  }

  if (!closestFrameIdx.has_value()) {
    return {};
  }
  // NOTE: the "closest keyframe too far in time" check is NOT applied globally
  // here. The frame-local odometry path (below) anchors on the requested
  // source's OWN fresh raw pose (last_raw_pose_by_source), not on this
  // keyframe, so it can still serve a valid short-term prediction when the
  // sparse central-map keyframes leave a gap > max_time_to_use_velocity_model
  // (common in SharedMapOnly mode after Phase-A keyframe-creation gating).
  // Gating the WHOLE function on keyframe proximity returned nullopt on those
  // gaps, which starved LidarOdometry's motion model ("Not able to use velocity
  // motion model") and eventually froze it on MulRan. The reference-frame path
  // and the no-raw-anchor fallback re-apply the proximity check themselves so
  // their behavior is unchanged.
  const bool closestWithinVelWindow =
    (*closestFrameDt <= params_.max_time_to_use_velocity_model);

  // 3) Recover the closest state (in the reference/map frame).
  NavState ret = get_latest_state_and_covariance(*closestFrameIdx);

  // Approximate twist uncertainty growth due to random walk:
  {
    auto twist_cov = ret.twist_inv_cov.inverse_LLt();
    for (int i = 0; i < 3; i++) {
      twist_cov(0 + i, 0 + i) +=
        mrpt::square(params_.sigma_random_walk_acceleration_linear * closestFrameDtSigned);
      twist_cov(3 + i, 3 + i) +=
        mrpt::square(params_.sigma_random_walk_acceleration_angular * closestFrameDtSigned);
    }
    ret.twist_inv_cov = twist_cov.inverse_LLt();
  }

  // 4) Produce the pose in the requested frame.
  if (frame_id == params_.reference_frame_name) {
    // The reference-frame path extrapolates the closest keyframe's {map} pose,
    // so it genuinely needs a keyframe within the velocity-model time window.
    if (!closestWithinVelWindow) {
      return {};
    }
    // Reference ({map}) frame.
    //
    // Phase C (high_rate_use_latest_sensors): scan last_raw_pose_by_source for
    // the freshest raw anchor, link it to the GTSAM-estimated T_map_to_odom for
    // that source (direct match OR the "_kf" counterpart by naming convention),
    // and compose to get a high-rate {map} estimate that tracks the source's
    // own odometry freshness rather than the sparse KF cadence (~1 Hz).
    // Falls back to the kinematic extrapolation if no suitable source found.
    if (params_.high_rate_use_latest_sensors && !state_.last_raw_pose_by_source.empty()) {
      const auto & idToName = state_.known_odom_frames.getInverseMap();
      const auto & nameToId = state_.known_odom_frames.getDirectMap();

      std::optional<mrpt::Clock::time_point> freshestStamp;
      std::optional<mrpt::poses::CPose3D> freshestPoseInMap;

      for (const auto & [rawFrameIdx, rawAnchor] : state_.last_raw_pose_by_source) {
        const double dtFromRaw = mrpt::system::timeDifference(rawAnchor.stamp, timestamp);
        if (std::abs(dtFromRaw) > params_.max_time_to_use_velocity_model) {
          continue;
        }
        if (freshestStamp.has_value() && rawAnchor.stamp <= *freshestStamp) {
          continue;  // not fresher than our current best
        }

        // Resolve T_map_to_odom: try direct match first, then "foo"→"foo_kf".
        const mrpt::poses::CPose3DPDFGaussian * pFrame = nullptr;
        auto itEst = state_.last_estimated_frames.find(rawFrameIdx);
        if (itEst != state_.last_estimated_frames.end()) {
          pFrame = &itEst->second;
        } else {
          const auto itName = idToName.find(rawFrameIdx);
          if (itName != idToName.end()) {
            const auto itKf = nameToId.find(itName->second + "_kf");
            if (itKf != nameToId.end()) {
              const auto itKfEst = state_.last_estimated_frames.find(itKf->second);
              if (itKfEst != state_.last_estimated_frames.end()) {
                pFrame = &itKfEst->second;
              }
            }
          }
        }
        if (pFrame == nullptr) {
          continue;
        }

        freshestStamp = rawAnchor.stamp;
        const auto poseInOdom =
          rawAnchor.pose.mean + body_twist_delta(params_, ret.twist, dtFromRaw);
        freshestPoseInMap = pFrame->mean + poseInOdom;
      }

      if (freshestPoseInMap.has_value()) {
        ret.pose.mean = *freshestPoseInMap;
        return ret;
      }
    }

    // Fallback: kinematic extrapolation from the closest sparse KF.
    ret.pose.mean = ret.pose.mean + body_twist_delta(params_, ret.twist, closestFrameDtSigned);
    return ret;
  }

  // Non-reference odometry frame {odom_i}: do NOT reconstruct the pose globally
  // as X(kf) (-) T_map_to_odom_i. Over the non-windowed central map that
  // reconstruction leaks every {map} correction (geo-ref / loop closure /
  // per-solve optimizer jitter) into the short-term prediction, producing
  // meter/degree jumps between consecutive scans that wreck a front end's ICP
  // initial guess (observed on MulRan right after geo-ref convergence). Instead
  // anchor on the source's OWN last raw pose in {odom_i} and extrapolate by the
  // body-twist increment: frame-local, and immune to {map} corrections.
  const auto it = state_.known_odom_frames.find_key(frame_id);
  if (it == state_.known_odom_frames.getDirectMap().end()) {
    MRPT_LOG_THROTTLE_WARN_FMT(
      5.0, "[estimated_navstate] Requested unknown odometry frame_id='%s'", frame_id.c_str());
    return {};
  }
  const auto requestedFrameIdx = it->second;

  const auto itRaw = state_.last_raw_pose_by_source.find(requestedFrameIdx);
  if (itRaw == state_.last_raw_pose_by_source.end()) {
    // No raw pose received from this source yet: fall back to the global
    // conversion (X(kf) (-) T_map_to_odom_i). This is correct while {map} and
    // {odom_i} still coincide, i.e. before any geo-ref / loop closure pulls the
    // map (the regime before the first fuse_pose() for this source lands).
    const auto itFrame = state_.last_estimated_frames.find(requestedFrameIdx);
    if (itFrame == state_.last_estimated_frames.end()) {
      return {};
    }
    // This fallback extrapolates the closest keyframe pose, so (as before the
    // gate relaxation) it requires that keyframe to be within the time window.
    if (!closestWithinVelWindow) {
      return {};
    }
    ret.pose.mean = ret.pose.mean + body_twist_delta(params_, ret.twist, closestFrameDtSigned);
    mrpt::poses::CPose3DPDFGaussianInf posePdfFrame_wrt_map_inf;
    posePdfFrame_wrt_map_inf.copyFrom(itFrame->second);
    ret.pose = ret.pose - posePdfFrame_wrt_map_inf;
    return ret;
  }

  // Frame-local extrapolation from the source's last raw pose in {odom_i}:
  const auto & rawAnchor = itRaw->second;
  const double dtPred = mrpt::system::timeDifference(rawAnchor.stamp, timestamp);

  if (std::abs(dtPred) > params_.max_time_to_use_velocity_model) {
    // This source's last raw pose is too old to extrapolate from.
    return {};
  }

  mrpt::poses::CPose3DPDFGaussian pred;
  pred.mean = rawAnchor.pose.mean + body_twist_delta(params_, ret.twist, dtPred);

  // Frame-local covariance: the anchor is the front end's own (near-exact) pose
  // in {odom_i}, so the prediction uncertainty is dominated by the one-step
  // extrapolation, NOT by the absolute {map}-frame keyframe covariance (which
  // grows unboundedly with gauge distance and would otherwise make callers
  // reject the motion model). Start from the (already floored) raw-pose
  // covariance and add a constant-acceleration displacement growth over |dt|.
  pred.cov = rawAnchor.pose.cov;
  const double adt = std::abs(dtPred);
  for (int i = 0; i < 3; i++) {
    pred.cov(i, i) += mrpt::square(params_.sigma_random_walk_acceleration_linear * adt * adt);
    pred.cov(3 + i, 3 + i) += mrpt::square(params_.sigma_random_walk_acceleration_angular * adt * adt);
  }

  ret.pose.copyFrom(pred);  // NavState.pose is CPose3DPDFGaussianInf

  static const bool tracePred = (::getenv("MOLA_MAPPER3D_TRACE_PREDICT") != nullptr);
  if (tracePred) {
    const auto disp = body_twist_delta(params_, ret.twist, dtPred);
    MRPT_LOG_INFO_FMT(
      "[PRED-TRACE] dt=%.3f twist_v=(%.2f,%.2f,%.2f) wz=%.3f |disp|=%.3f m  "
      "anchor_sigma_xyz=(%.3f,%.3f,%.3f) pred_sigma_xyz=(%.3f,%.3f,%.3f)",
      dtPred, ret.twist.vx, ret.twist.vy, ret.twist.vz, ret.twist.wz, disp.norm(),
      std::sqrt(rawAnchor.pose.cov(0, 0)), std::sqrt(rawAnchor.pose.cov(1, 1)),
      std::sqrt(rawAnchor.pose.cov(2, 2)), std::sqrt(pred.cov(0, 0)), std::sqrt(pred.cov(1, 1)),
      std::sqrt(pred.cov(2, 2)));
  }
  return ret;
}

std::optional<mrpt::poses::CPose3DPDFGaussian> Mapper3D::estimated_T_map_to_odometry_frame(
  const std::string & frame_id) const
{
  auto lck = mrpt::lockHelper(stateMutex_);

  const auto & str2id = state_.known_odom_frames.getDirectMap();
  const auto it = str2id.find(frame_id);
  if (it == str2id.end()) {
    return {};
  }
  const auto itFrame = state_.last_estimated_frames.find(it->second);
  if (itFrame == state_.last_estimated_frames.end()) {
    return {};
  }
  return {itFrame->second};
}

std::optional<mrpt::poses::CPose3DPDFGaussian> Mapper3D::estimated_T_enu_to_map() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  const auto it = state_.last_estimated_frames.find(REFERENCE_FRAME_ID);
  if (it == state_.last_estimated_frames.end()) {
    return {};
  }
  return {it->second};
}

std::optional<mola::Georeferencing> Mapper3D::current_georeferencing() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  return state_.geo_reference;
}

// ---------------------------------------------------------------------------
// Keyframe-neighbor helpers
// ---------------------------------------------------------------------------
Mapper3D::pair_nearby_frame_iterators_t Mapper3D::find_before_after(
  const mrpt::Clock::time_point & t, bool allow_exact_match) const
{
  const auto & m = state_.time_to_kf_id.getDirectMap();

  if (m.empty()) {
    return {m.end(), m.end()};
  }

  auto after = m.upper_bound(t);

  if (!allow_exact_match) {
    auto before = (after == m.begin()) ? m.end() : std::prev(after);
    if (before != m.end() && before->first == t) {
      auto element_before_match = (before == m.begin()) ? m.end() : std::prev(before);
      return {element_before_match, after};
    }
    return {before, after};
  }

  if (after == m.begin()) {
    return {m.end(), after};
  }
  auto before = std::prev(after);
  return {before, after};
}

std::optional<KeyFrameID> Mapper3D::pick_closest(
  const pair_nearby_frame_iterators_t & closestFrames, const mrpt::Clock::time_point & stamp) const
{
  const auto & m = state_.time_to_kf_id.getDirectMap();
  const auto & [before, after] = closestFrames;

  if (before == m.end() && after == m.end()) {
    return std::nullopt;
  }
  if (before == m.end()) {
    return after->second;
  }
  if (after == m.end()) {
    return before->second;
  }
  const double dtBefore = std::abs(mrpt::system::timeDifference(stamp, before->first));
  const double dtAfter = std::abs(mrpt::system::timeDifference(stamp, after->first));
  return (dtBefore < dtAfter) ? before->second : after->second;
}

std::optional<KeyFrameID> Mapper3D::find_nearest_kf_locked(
  const mrpt::Clock::time_point & t) const
{
  return pick_closest(find_before_after(t, true), t);
}

bool Mapper3D::sensor_kf_creation_allowed() const
{
  switch (params_.keyframe_creation_source) {
    case KeyframeCreationSource::SharedMapOnly:
      return false;
    case KeyframeCreationSource::SensorClock:
      return true;
    case KeyframeCreationSource::Auto:
    default:
      // Before any requestInsertKeyframe() arrives: legacy creation behavior.
      // After: behave like SharedMapOnly so the sparse LIO/VIO backbone governs
      // graph growth instead of every dense scan spawning a variable.
      return !shared_kf_producer_active_;
  }
}

}  // namespace mola::mapper_3d
