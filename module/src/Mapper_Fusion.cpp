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
 * @file   Mapper_Fusion.cpp
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
#include <mola_mapper/Mapper.h>
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
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
#include <numeric>

#include "GtsamData.h"
#include "covariance_utils.h"
#include "factor_builders.h"

namespace mola::mapper
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

/// Picks the velocity to extrapolate a source's raw {odom_i} anchor with:
/// the low-pass-filtered finite-difference twist when enabled and available,
/// otherwise the raw finite difference, otherwise `fallback` (the graph V/W).
const mrpt::math::TTwist3D & source_predict_twist(
  const Parameters & params, const WorldModelState::RawSourcePose & anchor,
  const mrpt::math::TTwist3D & fallback)
{
  if (params.predict_twist_filter_enabled && anchor.filtered_local_twist.value.has_value()) {
    return *anchor.filtered_local_twist.value;
  }
  if (anchor.has_local_twist) {
    return anchor.local_twist;
  }
  return fallback;
}

}  // namespace

// ---------------------------------------------------------------------------
// GTSAM (re)initialization
// ---------------------------------------------------------------------------
void Mapper::reinitialize_gtsam_locked()
{
  gtsam::ISAM2Params isam2Params;
  isam2Params.relinearizeThreshold = 0.1;
  isam2Params.relinearizeSkip = 1;
  state_.gtsam->isam2.emplace(isam2Params);

  // Always define the persistent T_enu_to_map variable, so it can be used for
  // gravity-alignment via IMU accelerometer even without GNSS.
  auto enu2map = gtsam::Pose3::Identity();
  gtsam::Matrix6 enu2map_cov = gtsam::Matrix6::Identity() * mrpt::square(ENU2MAP_WEAK_SIGMA);

  // T_enu_to_map is the transform between TWO gravity-aligned world frames (both
  // {enu} and {map} are level), so it is PHYSICALLY a level transform: a yaw
  // rotation plus a 3D translation, with roll == pitch == 0. We therefore pin
  // its roll/pitch TIGHT regardless of whether the geo-reference is estimated.
  // This is the crux of geo-ref stability: a FREE roll/pitch on T_enu_to_map
  // lets GNSS (or the gravity factor) satisfy itself "for free" by TILTING the
  // weakly-prior'd transform instead of flattening the drifted keyframe chain,
  // leaving {map} z/tilt-drifted and the keyframes uncorrected. With roll/pitch
  // pinned, the only way to satisfy the absolute factors is to bend the (soft)
  // keyframe chain back to level/true, which is what we want.
  //   - yaw: WEAK always (the geo-reference azimuth is the unknown; the IMU
  //     attitude or the GNSS trajectory shape drives it).
  //   - translation: TIGHT for pure odometry (map origin == ENU origin), but
  //     WEAK when estimating geo-ref (the ENU offset of the map origin is the
  //     unknown being solved for).
  // gtsam Pose3 tangent order: (roll, pitch, yaw, x, y, z).
  // NOTE: bending the keyframe chain does not corrupt LidarOdometry's ICP
  // initial guess ONLY because estimated_navstate(t,{odom_i}) is FULLY
  // frame-local: it anchors on the source's own raw pose (not reconstructed
  // through {map}) AND extrapolates with the source's own finite-difference
  // twist (RawSourcePose::local_twist), NOT the graph V/W. The graph twist is
  // re-optimized by these same absolute factors every solve, so feeding it to
  // the predictor would leak the chain-bending jitter back into LIO's ICP guess
  // (it did: ~0% goodness on DCC01 until the twist was made frame-local too).
  if (!params_.fixed_geo_reference.has_value()) {
    const double tight = params_.enu_to_map_prior_sigma_no_georef;
    const double transSigma = params_.estimate_geo_reference ? ENU2MAP_WEAK_SIGMA : tight;
    enu2map_cov = gtsam::Matrix6::Zero();
    enu2map_cov.diagonal() << mrpt::square(tight), mrpt::square(tight),
      mrpt::square(ENU2MAP_WEAK_SIGMA), mrpt::square(transSigma), mrpt::square(transSigma),
      mrpt::square(transSigma);
  }

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
OdometryFrameID Mapper::add_or_get_odom_frame_id_locked(const std::string & frame_id_name)
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
KeyFrameID Mapper::create_or_get_keyframe_by_timestamp_locked(
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

  // IMU rides the backend keyframe cadence: whenever ANY source creates a new
  // keyframe, drain the IMU buffer for the interval since the previous keyframe
  // and attach the gravity/attitude/gyro factors to it. IMU itself never
  // creates keyframes (see fuse_imu / emit_imu_factors_for_keyframe_locked).
  emit_imu_factors_for_keyframe_locked(newId);

  return newId;
}

void Mapper::initialize_new_frame(
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

void Mapper::add_kinematic_factor_between(KeyFrameID from, KeyFrameID to)
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
void Mapper::fuse_pose(
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

void Mapper::fuse_pose_locked(
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
  //
  // Also derive the body-frame twist from THIS source's consecutive raw poses
  // (finite difference in {odom_i}). The short-term {odom_i} prediction must be
  // FULLY frame-local: the per-keyframe graph V(kf)/W(kf) is re-optimized every
  // solve by the absolute factors (GNSS / IMU-gravity leveling / loop closure),
  // so it jitters (especially once T_enu_to_map roll/pitch is pinned and those
  // factors bend the soft keyframe chain) and would leak meter/degree jumps into
  // the prediction. The source's own finite-difference velocity is immune to
  // those {map}-frame corrections. See WorldModelState::RawSourcePose.
  WorldModelState::RawSourcePose newAnchor{timestamp, poseSanitized, {}, false, {}};
  if (const auto itPrev = state_.last_raw_pose_by_source.find(frame_id_idx);
      itPrev != state_.last_raw_pose_by_source.end()) {
    // Carry the filter state across updates (the anchor struct is replaced
    // wholesale below).
    newAnchor.filtered_local_twist = itPrev->second.filtered_local_twist;

    const double dt = mrpt::system::timeDifference(itPrev->second.stamp, timestamp);
    if (dt > 1e-4 && dt <= params_.max_time_to_use_velocity_model) {
      // Relative body motion prev^-1 (+) curr, then log() / dt -> body twist.
      const mrpt::poses::CPose3D rel = poseSanitized.mean - itPrev->second.pose.mean;
      const auto logv = mrpt::poses::Lie::SE<3>::log(rel);
      newAnchor.local_twist = mrpt::math::TTwist3D(
        logv[0] / dt, logv[1] / dt, logv[2] / dt, logv[3] / dt, logv[4] / dt, logv[5] / dt);
      newAnchor.has_local_twist = true;

      // Damp this single-interval finite difference before it reaches the
      // front end's motion prior.
      if (params_.predict_twist_filter_enabled) {
        newAnchor.filtered_local_twist.update(
          newAnchor.local_twist, timestamp, params_.predict_twist_filter_time_const);
      }
    }
  }
  state_.last_raw_pose_by_source[frame_id_idx] = newAnchor;

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

void Mapper::link_into_odometry_chain_locked(
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
          itSt->second.pose = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(init));
        }
      }
    }
  }
}

void Mapper::add_odom_chain_edge_locked(KeyFrameID a, KeyFrameID b)
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
  gtsam::Vector6 sigmasXYZYPR = relPdf.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();
  sigmasXYZYPR *= params_.odometry_edge_uncertainty_multiplier;
  for (int k = 0; k < 3; k++) {
    sigmasXYZYPR[k] += params_.odometry_edge_min_sigma_xyz;
    sigmasXYZYPR[3 + k] += mrpt::DEG2RAD(params_.odometry_edge_min_sigma_ang_deg);
  }
  // Anisotropic roll/pitch compliance: a positive value RAISES the roll/pitch
  // floor (indices 5=roll, 4=pitch) while leaving yaw (index 3) at the tight
  // floor above. This lets the absolute IMU gravity factor bend the otherwise
  // near-rigid LIO orientation chain back to level WITHOUT freeing the yaw gauge
  // (which has no absolute reference when there is no GNSS, and whose loss makes
  // the trajectory wander). Only meaningful with a trustworthy gravity factor,
  // i.e. imu_use_filtered_gravity (see agents.md "IMU gravity leveling").
  if (params_.odometry_edge_min_sigma_rollpitch_deg > 0) {
    const double rp = mrpt::DEG2RAD(params_.odometry_edge_min_sigma_rollpitch_deg);
    sigmasXYZYPR[4] = std::max(sigmasXYZYPR[4], rp);  // pitch
    sigmasXYZYPR[5] = std::max(sigmasXYZYPR[5], rp);  // roll
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

void Mapper::fuse_odometry(
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

void Mapper::fuse_twist(
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

void Mapper::fuse_imu(const mrpt::obs::CObservationIMU & imu)
{
  const ProfilerEntry tle(profiler_, "fuse_imu");
  auto lck = mrpt::lockHelper(stateMutex_);
  // fuse_imu NEVER touches the graph: it only moves the reading to the vehicle
  // frame and accumulates it in the LocalVelocityBuffer. The backend factors
  // are built later, once per real keyframe, in
  // emit_imu_factors_for_keyframe_locked() (hooked from keyframe creation).
  // This is the IMU's short-term-evidence store; the predictor side reads from
  // the same buffer (future work, plan 4.12). No notify_optimizer() here: no
  // pending factor/value was produced.
  ingest_imu_sample_locked(imu);
}

void Mapper::ingest_imu_sample_locked(const mrpt::obs::CObservationIMU & imu)
{
  // Move the raw reading to the vehicle "base_link" frame with a per-sensor,
  // STATEFUL ImuTransformer (rotation + rigid lever-arm/centripetal correction
  // a_body = R*a_imu - alpha x t - w x (w x t)). One transformer per sensor
  // label, since it keeps angular-velocity/acceleration low-pass state.
  auto & transformer = imu_transformers_[imu.sensorLabel];
  const mrpt::obs::CObservationIMU bodyImu = transformer.process(imu);

  // After process() the reading is expressed at the vehicle frame (sensorPose
  // is identity), so the factors built from the buffer use an identity sensor
  // pose and the orientation we store is the VEHICLE attitude in the world.
  const mola::imu::TimeStamp t = mrpt::Clock::toDouble(bodyImu.timestamp);

  if (
    imu.has(mrpt::obs::IMU_X_ACC) && imu.has(mrpt::obs::IMU_Y_ACC) &&
    imu.has(mrpt::obs::IMU_Z_ACC)) {
    // ROTATION-ONLY accel for the gravity buffer (NOT the lever-arm-corrected
    // bodyImu accel). Gravity DIRECTION is invariant to a rigid sensor offset,
    // and the ImuTransformer's lever-arm term (-alpha x t - w x (w x t)) turns
    // this IMU's noisy gyro into several m/s^2 of direction scatter over its
    // ~0.44 m lever arm, destroying otherwise-clean stationary gravity windows
    // (the very low-dynamics epochs the leveling relies on). The low-dynamics
    // gates + spread gate already reject genuine motion accel, so the lever-arm
    // correction is not needed here and is actively harmful.
    const auto aSensor = mrpt::math::TVector3D(
      imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC), imu.get(mrpt::obs::IMU_Z_ACC));
    const auto aBodyRot = imu.sensorPose.rotateVector(aSensor);
    imu_buffer_.add_linear_acceleration(t, aBodyRot);
  }
  if (bodyImu.has(mrpt::obs::IMU_WX)) {
    imu_buffer_.add_angular_velocity(
      t, {bodyImu.get(mrpt::obs::IMU_WX), bodyImu.get(mrpt::obs::IMU_WY),
          bodyImu.get(mrpt::obs::IMU_WZ)});
  }
  if (imu.has(mrpt::obs::IMU_ORI_QUAT_W)) {
    mrpt::math::CQuaternionDouble q;
    q.w(imu.get(mrpt::obs::IMU_ORI_QUAT_W));
    q.x(imu.get(mrpt::obs::IMU_ORI_QUAT_X));
    q.y(imu.get(mrpt::obs::IMU_ORI_QUAT_Y));
    q.z(imu.get(mrpt::obs::IMU_ORI_QUAT_Z));
    // Reject a PLACEHOLDER orientation. Some drivers (e.g. the Hesai built-in
    // IMU) do NOT estimate attitude yet still ship a bit-exact identity
    // quaternion tagged orientation_covariance[0]=0, which the ROS bridge
    // forwards as a "valid" orientation. Trusting it is actively harmful: it is
    // NOT the vehicle attitude, so once brought to the vehicle frame through a
    // rotated mount (R_world_vehicle = I * R_vehicle_sensor^T) it becomes a
    // bogus tilt, and the absolute-attitude factor rolls the whole map (the
    // real floor turns into a wall on the Hesai "_90deg" bag). A genuine
    // attitude estimate is essentially never bit-exact identity, and a real
    // attitude-capable IMU on a rotated mount reports a NON-identity world
    // quaternion even when the vehicle is level, so a (near) identity reading
    // is an unambiguous placeholder signature. See test-imu-gnss-fusion.cpp
    // (test_imu_rotated_mount_orientation).
    const bool isPlaceholderIdentity = std::abs(q.x()) < 1e-6 && std::abs(q.y()) < 1e-6 &&
                                       std::abs(q.z()) < 1e-6 &&
                                       std::abs(std::abs(q.w()) - 1.0) < 1e-6;
    if (
      !std::isnan(q.w()) && !std::isnan(q.x()) && !std::isnan(q.y()) && !std::isnan(q.z()) &&
      std::abs(q.norm() - 1.0) <= 0.02 && !isPlaceholderIdentity) {
      // Sensor's measured world attitude R_world_sensor, brought to the vehicle
      // frame: R_world_vehicle = R_world_sensor * R_vehicle_sensor^T. This keeps
      // all sensor-mount handling at ingest time, mirroring ImuTransformer.
      mrpt::math::CMatrixDouble33 R_world_sensor;
      q.rotationMatrixNoResize(R_world_sensor);
      const auto R_vehicle_sensor = imu.sensorPose.getRotationMatrix();
      mola::imu::SO3 R_world_vehicle;
      R_world_vehicle.asEigen() = R_world_sensor.asEigen() * R_vehicle_sensor.asEigen().transpose();
      imu_buffer_.add_orientation(t, R_world_vehicle);
    } else if (isPlaceholderIdentity) {
      MRPT_LOG_THROTTLE_WARN(
        5.0,
        "Ignoring placeholder identity IMU orientation quaternion (sensor reports "
        "no real attitude); using accelerometer gravity for leveling only.");
    } else {
      MRPT_LOG_THROTTLE_WARN(5.0, "Ignoring invalid/NaN IMU orientation quaternion");
    }
  }

  // Bounded-rate gravity leveling from the IMU stream (fires during stops, when
  // no keyframe is created but the accelerometer sees clean gravity).
  maybe_emit_gravity_factor_locked(t);
}

void Mapper::maybe_emit_gravity_factor_locked(mola::imu::TimeStamp tNow)
{
  // No keyframe yet -> nothing to attach a gravity factor to.
  if (state_.empty()) {
    return;
  }
  const mola::imu::TimeStamp gWin = params_.imu_gravity_window_sec;

  // Bounded rate: evaluate the window at most once every gWin seconds, so
  // consecutive emitted factors use NON-overlapping (independent) windows.
  if (last_gravity_check_t_.has_value() && (tNow - *last_gravity_check_t_) < gWin) {
    return;
  }
  last_gravity_check_t_ = tNow;

  // At most ONE gravity factor per keyframe: a long stop keeps the same latest
  // keyframe and must not pile up dozens of constraints on the same pose.
  const KeyFrameID latestKf = state_.last_kf_id();
  if (last_gravity_kf_id_.has_value() && *last_gravity_kf_id_ == latestKf) {
    return;
  }

  // Reduce the recent proper-acceleration window to a robust, low-dynamics
  // gravity direction (the filter rejects motion-contaminated samples and whole
  // windows whose survivors disagree). A motion-canceling window of length gWin
  // makes the mean residual acceleration ~(v_end - v_start)/gWin, which is small
  // for steady or stop-and-go driving; the clean stops dominate the survivors.
  const auto window = imu_buffer_.window_since(tNow - gWin, tNow);
  if (window.a_b.empty()) {
    return;
  }
  imu_gravity_filter_.clear();
  for (const auto & [ta, a] : window.a_b) {
    std::optional<mrpt::math::TVector3D> w;
    if (const auto itw = window.w_b.find(ta); itw != window.w_b.end()) {
      w = itw->second;
    }
    imu_gravity_filter_.addSample(mrpt::Clock::fromDouble(ta), a, w);
  }
  const auto est = imu_gravity_filter_.flush();
  if (!est.has_value()) {
    return;
  }
  last_gravity_kf_id_ = latestKf;

  // Accel is already in the vehicle frame (ImuTransformer at ingest), so the
  // factor takes an identity sensor pose. The latest keyframe's orientation
  // matches the (stationary) vehicle attitude this gravity was measured at.
  const gtsam::Vector3 g = {
    est->gravity_body_normalized.x, est->gravity_body_normalized.y, est->gravity_body_normalized.z};
  auto accNoise = gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(est->sigma_deg));
  state_.gtsam->newFactors.emplace_shared<mola::factors::MeasuredGravityFactor>(
    symbol_T_enu_to_map, T(latestKf), gtsam::Pose3::Identity(), g, accNoise);
  geo_ref_counters_.imu_gravity++;
  notify_optimizer();

  thread_local const bool traceGeom = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_GEOM", false);
  if (traceGeom) {
    static mrpt::math::TVector3D gAccum{0, 0, 0};
    static size_t gN = 0;
    gAccum = gAccum + est->gravity_body_normalized;
    gN++;
    auto m = gAccum * (1.0 / static_cast<double>(gN));
    const double mn = m.norm();
    if (mn > 1e-9) {
      m = m * (1.0 / mn);
    }
    MRPT_LOG_THROTTLE_WARN_FMT(
      1.0,
      "[GRAV-BODY-TRACE] n=%zu cur up_veh=(%.04f,%.04f,%.04f) mean up_vehicle=(%.4f,%.4f,%.4f) "
      "-> systematic pitch-lean=%.2f deg roll-lean=%.2f deg",
      gN, est->gravity_body_normalized.x, est->gravity_body_normalized.y,
      est->gravity_body_normalized.z, m.x, m.y, m.z, mrpt::RAD2DEG(std::atan2(m.x, m.z)),
      mrpt::RAD2DEG(std::atan2(m.y, m.z)));
  }
}

void Mapper::emit_imu_factors_for_keyframe_locked(KeyFrameID newKf)
{
  // First IMU keyframe: nothing accumulated behind it yet. Anchor and return.
  if (!last_imu_kf_.has_value()) {
    last_imu_kf_ = newKf;
    return;
  }
  const KeyFrameID prevKf = *last_imu_kf_;
  last_imu_kf_ = newKf;

  const mola::imu::TimeStamp tFrom = mrpt::Clock::toDouble(state_.time_to_kf_id.inverse(prevKf));
  const mola::imu::TimeStamp tTo = mrpt::Clock::toDouble(state_.time_to_kf_id.inverse(newKf));
  if (!(tTo > tFrom)) {
    return;
  }
  const auto window = imu_buffer_.window_since(tFrom, tTo);

  const auto identitySensor = gtsam::Pose3::Identity();

  // (1) Gravity-leveling factors are NOT emitted here. Keyframes are created from
  // distance traveled, so they never land during the STOPS where the
  // accelerometer sees clean, motion-free gravity; hooking gravity to keyframe
  // creation would only ever sample the moving (motion-contaminated) epochs and
  // TILT the map. Gravity is emitted instead from the IMU stream at a bounded
  // rate, attached to the latest keyframe (see maybe_emit_gravity_factor_locked).

  // (2) Absolute attitude / azimuth observation (vehicle world attitude). Now
  // added ALWAYS (when available), not only under geo-referencing: the absolute
  // attitude IS an azimuth reference, so feeding it lets iSAM2 estimate the
  // geo-reference (T_enu_to_map yaw) automatically even with GNSS off. The
  // no-geo-ref F0 prior is yaw-free (see reinitialize_gtsam_locked) so this
  // azimuth lands on T_enu_to_map instead of fighting the pinned map. Uses the
  // LATEST orientation in the interval; identity sensor pose (already in body).
  if (params_.imu_attitude_sigma_deg > 0 && !window.q.empty()) {
    const mola::imu::SO3 & R = window.q.rbegin()->second;
    auto measuredRotation = gtsam::Rot3(R.asEigen());
    // ENU has yaw=0 => East; correct to azimuth wrt true North:
    measuredRotation =
      gtsam::Rot3::Rz(mrpt::DEG2RAD(90.0 + params_.imu_attitude_azimuth_offset_deg)) *
      measuredRotation;
    auto rotationNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params_.imu_attitude_sigma_deg));
    state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationFactor>(
      symbol_T_enu_to_map, T(newKf), identitySensor, measuredRotation, rotationNoise);
    geo_ref_counters_.imu_attitude++;
  }

  // (3) Optional gyroscope angular-velocity prior (averaged over the interval,
  // already in the vehicle/body frame):
  if (params_.imu_angular_velocity_sigma_deg > 0 && !window.w_b.empty()) {
    gtsam::Vector3 wSum = gtsam::Vector3::Zero();
    for (const auto & [tw, w] : window.w_b) {
      wSum += gtsam::Vector3(w.x, w.y, w.z);
    }
    const gtsam::Vector3 wBody = wSum / static_cast<double>(window.w_b.size());
    auto wNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params_.imu_angular_velocity_sigma_deg));
    state_.gtsam->newFactors.addPrior(W(newKf), wBody, wNoise);
    geo_ref_counters_.imu_omega++;
  }

  MRPT_LOG_THROTTLE_DEBUG_FMT(
    5.0,
    "[fuse_imu] kf=%zu (interval %zu accel / %zu ori samples) factors so far: grav=%zu att=%zu "
    "omega=%zu",
    static_cast<size_t>(newKf), window.a_b.size(), window.q.size(), geo_ref_counters_.imu_gravity,
    geo_ref_counters_.imu_attitude, geo_ref_counters_.imu_omega);
}

void Mapper::trace_imu_factors_locked(const gtsam::Values & estimate)
{
  thread_local const bool traceImu = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_IMU", false);
  if (!traceImu || !state_.gtsam->isam2.has_value()) {
    return;
  }
  try {
    if (estimate.exists(symbol_T_enu_to_map)) {
      const auto Tem = mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(estimate.at<gtsam::Pose3>(symbol_T_enu_to_map)));
      MRPT_LOG_WARN_FMT(
        "[IMU-TRACE] T_enu_to_map(F0): xyz=(%.2f,%.2f,%.2f) roll=%.3f pitch=%.3f yaw=%.3f deg",
        Tem.x(), Tem.y(), Tem.z(), mrpt::RAD2DEG(Tem.roll()), mrpt::RAD2DEG(Tem.pitch()),
        mrpt::RAD2DEG(Tem.yaw()));
    }
    const auto & fg = state_.gtsam->isam2->getFactorsUnsafe();
    std::vector<double> gravDeg;
    double gravChi2 = 0;
    gtsam::Vector3 residSum = gtsam::Vector3::Zero();
    for (const auto & f : fg) {
      if (!f) {
        continue;
      }
      if (dynamic_cast<const mola::factors::MeasuredGravityFactor *>(f.get()) != nullptr) {
        gravChi2 += f->error(estimate);
        const auto * nmf = dynamic_cast<const gtsam::NoiseModelFactor *>(f.get());
        if (nmf != nullptr) {
          const auto e = nmf->unwhitenedError(estimate);
          gravDeg.push_back(mrpt::RAD2DEG(e.norm()));
          residSum += e;
        }
      }
    }
    if (gravDeg.empty()) {
      return;
    }
    std::sort(gravDeg.begin(), gravDeg.end());
    const double min = gravDeg.front();
    const double mean =
      std::accumulate(gravDeg.begin(), gravDeg.end(), 0.0) / static_cast<double>(gravDeg.size());
    const double median = gravDeg[gravDeg.size() / 2];
    const double p90 = gravDeg[(gravDeg.size() * 9) / 10];
    const double meanVecNorm =
      mrpt::RAD2DEG((residSum / static_cast<double>(gravDeg.size())).norm());
    MRPT_LOG_WARN_FMT(
      "[IMU-TRACE] gravity-resid(deg): n=%zu min=%.2f mean=%.2f median=%.2f p90=%.2f max=%.2f "
      "sum_chi2=%.0f | mean-VECTOR-norm=%.2f deg (systematic if ~mean; random/motion if <<mean)",
      gravDeg.size(), min, mean, median, p90, gravDeg.back(), gravChi2, meanVecNorm);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("[IMU-TRACE] failed:\n" << e.what());
  }
}

void Mapper::trace_keyframe_geometry_locked(const gtsam::Values & estimate)
{
  thread_local const bool traceGeom = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_GEOM", false);
  if (!traceGeom) {
    return;
  }
  try {
    // Walk the keyframes in TIME order and collect their {map}-frame positions
    // + body up-axis (3rd column of the rotation = vehicle "up" expressed in
    // {map}). The gravity factor levels ORIENTATION; this measures whether the
    // resulting POSITION path is actually level too.
    std::vector<mrpt::math::TPoint3D> pos;
    double upDotZsum = 0.0;  // average alignment of body up-axis with map +Z
    double pitchSum = 0.0;   // SIGNED mean keyframe pitch (deg): systematic nose-up?
    double rollSum = 0.0;    // SIGNED mean keyframe roll (deg)
    size_t nUp = 0;
    // RAW LIO odom z range (the un-leveled input chain), to tell apart a biased
    // gravity reference (graph z-drift << raw LIO z-drift => factor IS helping)
    // from the factor being outvoted (graph z-drift ~ raw LIO z-drift).
    double rawZmin = std::numeric_limits<double>::infinity();
    double rawZmax = -std::numeric_limits<double>::infinity();
    double rawPitchSum = 0.0;
    size_t nRaw = 0;
    for (const auto & [t, id] : state_.time_to_kf_id.getDirectMap()) {
      if (!estimate.exists(T(id))) {
        continue;
      }
      const mrpt::poses::CPose3D p(
        mrpt::gtsam_wrappers::toTPose3D(estimate.at<gtsam::Pose3>(T(id))));
      pos.emplace_back(p.x(), p.y(), p.z());
      // Body up-axis = R * (0,0,1) = third column of the rotation matrix:
      const auto & R = p.getRotationMatrix();
      const double upz = R(2, 2);  // z-component of the body up-axis in {map}
      upDotZsum += upz;
      pitchSum += mrpt::RAD2DEG(p.pitch());
      rollSum += mrpt::RAD2DEG(p.roll());
      nUp++;
      if (const auto itRaw = kf_odom_abs_pose_.find(id); itRaw != kf_odom_abs_pose_.end()) {
        const double rz = itRaw->second.mean.z();
        rawZmin = std::min(rawZmin, rz);
        rawZmax = std::max(rawZmax, rz);
        rawPitchSum += mrpt::RAD2DEG(itRaw->second.mean.pitch());
        nRaw++;
      }
    }
    if (pos.size() < 2) {
      return;
    }

    // z-span of the keyframe POSITIONS:
    double zmin = pos.front().z;
    double zmax = pos.front().z;
    for (const auto & q : pos) {
      zmin = std::min(zmin, q.z);
      zmax = std::max(zmax, q.z);
    }

    // Best-fit slope of z vs cumulative HORIZONTAL arc length: the average tilt
    // the path "climbs" as it moves. atan(slope) is the apparent path tilt.
    double arc = 0.0;
    std::vector<double> s;
    s.reserve(pos.size());
    s.push_back(0.0);
    for (size_t i = 1; i < pos.size(); ++i) {
      const double dx = pos[i].x - pos[i - 1].x;
      const double dy = pos[i].y - pos[i - 1].y;
      arc += std::hypot(dx, dy);
      s.push_back(arc);
    }
    // Least-squares slope of z = a*s + b:
    const double n = static_cast<double>(pos.size());
    double sumS = 0;
    double sumZ = 0;
    double sumSS = 0;
    double sumSZ = 0;
    for (size_t i = 0; i < pos.size(); ++i) {
      sumS += s[i];
      sumZ += pos[i].z;
      sumSS += s[i] * s[i];
      sumSZ += s[i] * pos[i].z;
    }
    const double denom = (n * sumSS - sumS * sumS);
    double slope = 0.0;
    if (std::abs(denom) > 1e-9) {
      slope = (n * sumSZ - sumS * sumZ) / denom;
    }
    const double tiltDeg = mrpt::RAD2DEG(std::atan(slope));
    const double upTiltDeg =
      (nUp > 0)
        ? mrpt::RAD2DEG(std::acos(std::clamp(upDotZsum / static_cast<double>(nUp), -1.0, 1.0)))
        : 0.0;

    const double meanPitch = (nUp > 0) ? pitchSum / static_cast<double>(nUp) : 0.0;
    const double meanRoll = (nUp > 0) ? rollSum / static_cast<double>(nUp) : 0.0;
    const double rawZspan = (nRaw > 0) ? (rawZmax - rawZmin) : 0.0;
    const double rawMeanPitch = (nRaw > 0) ? rawPitchSum / static_cast<double>(nRaw) : 0.0;

    MRPT_LOG_WARN_FMT(
      "[GEOM-TRACE] kfs=%zu arc=%.0f m | path-z: min=%.2f max=%.2f span=%.2f m | "
      "z-vs-arc slope=%.4f (path-tilt=%.2f deg) | mean body-up vs map+Z=%.2f deg | "
      "mean SIGNED kf pitch=%.2f roll=%.2f deg | RAW-LIO z-span=%.2f m mean-pitch=%.2f deg",
      pos.size(), arc, zmin, zmax, zmax - zmin, slope, tiltDeg, upTiltDeg, meanPitch, meanRoll,
      rawZspan, rawMeanPitch);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("[GEOM-TRACE] failed:\n" << e.what());
  }
}

void Mapper::fuse_gnss(const mrpt::obs::CObservationGPS & gps)
{
  const ProfilerEntry tle(profiler_, "fuse_gnss");
  auto lck = mrpt::lockHelper(stateMutex_);

  // Diagnostic: dump EVERY raw GNSS reading (and why it is accepted/rejected),
  // un-throttled, when MOLA_MAPPER_TRACE_GPS is set. Lets us inspect the data
  // quality (height constancy + per-fix ENU covariance) end to end.
  thread_local const bool traceGps = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_GPS", false);
  geo_ref_counters_.gnss_readings_seen++;
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
      "[GPS-TRACE] #%zu t=%.3f %s %s", static_cast<size_t>(geo_ref_counters_.gnss_readings_seen),
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
  geo_ref_counters_.gnss++;

  if (traceGps) {
    MRPT_LOG_INFO_FMT(
      "[GPS-TRACE] #%zu ACCEPTED -> kf=%zu ENU=(%.2f, %.2f, %.2f) sigma_enu=(%.2f, %.2f, %.2f) "
      "factors=%zu",
      static_cast<size_t>(geo_ref_counters_.gnss_readings_seen), static_cast<size_t>(this_kf_id),
      ENU_point.x, ENU_point.y, ENU_point.z, std::sqrt((*gps.covariance_enu)(0, 0)),
      std::sqrt((*gps.covariance_enu)(1, 1)), std::sqrt((*gps.covariance_enu)(2, 2)),
      geo_ref_counters_.gnss);
  }
  MRPT_LOG_THROTTLE_DEBUG_FMT(
    2.0, "[fuse_gnss] kf=%zu ENU=(%.2f, %.2f, %.2f) fix_quality=%d factors=%zu",
    static_cast<size_t>(this_kf_id), ENU_point.x, ENU_point.y, ENU_point.z,
    static_cast<int>(gga.fields.fix_quality), geo_ref_counters_.gnss);

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
void Mapper::optimize_and_refresh()
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

  trace_imu_factors_locked(localEstimate);
  trace_keyframe_geometry_locked(localEstimate);

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
      t.pose = mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(localEstimate.at<gtsam::Pose3>(T(id))));
      const auto linV = localEstimate.at<gtsam::Vector3>(V(id));
      const auto angV = localEstimate.at<gtsam::Vector3>(W(id));
      t.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};
      if (params_.enforce_planar_motion) {
        enforce_planar_pose(t.pose);
        enforce_planar_twist(t.twist);
      }
      thread_local const bool traceVW = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_VW", false);
      if (traceVW && (angV.norm() > 5.0 || linV.norm() < 0.5)) {
        // dt to the time-adjacent previous keyframe (the kinematic-factor dt):
        double dtPrev = -1.0;
        const auto & m = state_.time_to_kf_id.getDirectMap();
        if (auto itT = m.find(state_.time_to_kf_id.inverse(id));
            itT != m.end() && itT != m.begin()) {
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

        // Drive the predict-twist low-pass with the newest keyframe's optimized
        // twist, so the short-term prediction extrapolates a damped velocity
        // rather than this (least-constrained, per-solve jittery) node's raw one.
        if (params_.predict_twist_filter_enabled) {
          state_.filtered_predict_twist.update(
            it->second.twist, state_.time_to_kf_id.inverse(latestKfId),
            params_.predict_twist_filter_time_const);
        }
      }
    }
    std::string driftTrace;
    const auto & idToName = state_.known_odom_frames.getInverseMap();
    for (const auto & [fid, pdf] : tmpFrames) {
      // Publish the transform under the REAL odometry frame the front end uses
      // to BOTH query estimated_navstate() and draw its dense clouds / local map
      // (its publish_reference_frame, e.g. "odom"). The keyframes that DEFINE
      // this transform are registered under a dedicated "<name>_kf"
      // SharedKeyframeMap source (so the dense fuse_pose() anchor tie and the
      // keyframe ties never collide on one graph variable), and in SharedMapOnly
      // mode ONLY that "_kf" source creates keyframes -- so the transform is
      // computed there. But "<name>_kf" is an internal alias with no
      // visualization geometry of its own. Remap it to the base "<name>" so the
      // movable viz frame node LIO draws its local map under (and the ROS
      // map->odom /tf for "<name>", via estimated_T_map_to_odometry_frame) is
      // actually repositioned; otherwise that frame stays at identity and LIO's
      // local map sits at the origin while the {map} keyframe path is correctly
      // geo-referenced. The two names denote the SAME physical odometry frame, so
      // the transform is identical.
      auto targetFid = fid;
      const auto itName = idToName.find(fid);
      std::string nm = (itName != idToName.end()) ? itName->second : std::to_string(fid);
      static const std::string kKfSuffix = "_kf";
      if (
        nm.size() > kKfSuffix.size() &&
        nm.compare(nm.size() - kKfSuffix.size(), kKfSuffix.size(), kKfSuffix) == 0) {
        const std::string baseName = nm.substr(0, nm.size() - kKfSuffix.size());
        const auto itBase = state_.known_odom_frames.find_key(baseName);
        if (itBase != state_.known_odom_frames.getDirectMap().end()) {
          targetFid = itBase->second;
          nm = baseName;
        }
      }
      state_.last_estimated_frames[targetFid] = pdf;
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
      if (!geo_ref_counters_.georef_converged_announced) {
        geo_ref_counters_.georef_converged_announced = true;
        const auto [posSigma, oriSigmaDeg] =
          max_pos_and_orientation_sigma(tmpGeoRef->T_enu_to_map.cov);
        const auto & m = tmpGeoRef->T_enu_to_map.mean;
        MRPT_LOG_INFO_FMT(
          "[geo-ref] CONVERGED: T_enu_to_map=(%.2f, %.2f, %.2f, yaw=%.2f deg) "
          "sigma_pos=%.3f m sigma_ori=%.3f deg after %zu GNSS factors.",
          m.x(), m.y(), m.z(), mrpt::RAD2DEG(m.yaw()), posSigma, oriSigmaDeg,
          geo_ref_counters_.gnss);
      }
      state_.geo_reference = tmpGeoRef;
    }
  }
}

NavState Mapper::get_latest_state_and_covariance(KeyFrameID idx) const
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

std::optional<NavState> Mapper::estimated_navstate(
  const mrpt::Clock::time_point & timestamp, const std::string & frame_id)
{
  // This is queried at high rate by front ends from their own threads, and its
  // [[nodiscard]] std::optional<NavState> signature already promises a graceful
  // "not ready yet" for a transiently under-constrained / not-yet-solved graph.
  // Letting an exception escape instead would kill the calling module's thread
  // and take down fusion for the rest of the run.
  try {
    return estimated_navstate_impl(timestamp, frame_id);
  } catch (const std::exception & e) {
    MRPT_LOG_THROTTLE_ERROR_STREAM(
      2.0, "[estimated_navstate] Returning no estimate after exception:\n"
             << e.what());
    return {};
  }
}

std::optional<NavState> Mapper::estimated_navstate_impl(
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

  // A keyframe exists in time_to_kf_id as soon as it is CREATED, but its state
  // only lands in last_estimated_states after a solve commits. With the
  // background optimizer that window is routinely open, so treat it as the
  // ordinary "not ready yet" case rather than letting the lookup below throw.
  if (state_.last_estimated_states.count(*closestFrameIdx) == 0) {
    MRPT_LOG_THROTTLE_DEBUG_FMT(
      5.0, "[estimated_navstate] Keyframe %u not solved yet.",
      static_cast<unsigned>(*closestFrameIdx));
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
  const bool closestWithinVelWindow = (*closestFrameDt <= params_.max_time_to_use_velocity_model);

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

  // Extrapolate the low-pass-filtered velocity instead of the newest keyframe's
  // raw optimized twist. Only when anchoring on that newest keyframe: older
  // keyframes are constrained on both sides, so their twist is not the noisy
  // boundary estimate the filter exists to damp.
  if (
    params_.predict_twist_filter_enabled && state_.filtered_predict_twist.value.has_value() &&
    *closestFrameIdx == state_.last_kf_id()) {
    ret.twist = *state_.filtered_predict_twist.value;
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
        // Extrapolate in the source's OWN frame with its OWN finite-difference
        // twist (frame-local, immune to the graph V/W re-optimization jitter the
        // absolute factors inject), then compose into {map}.
        const mrpt::math::TTwist3D & twistOdom =
          source_predict_twist(params_, rawAnchor, ret.twist);
        const auto poseInOdom =
          rawAnchor.pose.mean + body_twist_delta(params_, twistOdom, dtFromRaw);
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

  // Frame-local twist: prefer the source's OWN finite-difference velocity over
  // the graph V/W. The graph twist is re-optimized every solve by the absolute
  // factors (GNSS / IMU-gravity leveling / loop closure); once T_enu_to_map
  // roll/pitch is pinned those factors bend the soft keyframe chain, so the
  // latest keyframe's V/W swings per solve and would otherwise leak meter/degree
  // jumps into this prediction (wrecking the front end's ICP guess on MulRan).
  ret.twist = source_predict_twist(params_, rawAnchor, ret.twist);

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
    pred.cov(3 + i, 3 + i) +=
      mrpt::square(params_.sigma_random_walk_acceleration_angular * adt * adt);
  }

  ret.pose.copyFrom(pred);  // NavState.pose is CPose3DPDFGaussianInf

  thread_local const bool tracePred = mrpt::get_env<bool>("MOLA_MAPPER_TRACE_PREDICT", false);
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

std::optional<mrpt::poses::CPose3DPDFGaussian> Mapper::estimated_T_map_to_odometry_frame(
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

std::optional<mrpt::poses::CPose3DPDFGaussian> Mapper::estimated_T_enu_to_map() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  const auto it = state_.last_estimated_frames.find(REFERENCE_FRAME_ID);
  if (it == state_.last_estimated_frames.end()) {
    return {};
  }
  return {it->second};
}

std::optional<mola::Georeferencing> Mapper::current_georeferencing() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  return state_.geo_reference;
}

void Mapper::set_geo_reference(const mola::Georeferencing & georef)
{
  bool rebuilt = false;
  {
    auto lck = mrpt::lockHelper(stateMutex_);

    MRPT_LOG_INFO_STREAM(
      "[set_geo_reference] Anchoring T_enu_to_map to a fixed, known value: "
      << georef.T_enu_to_map.mean.asString());

    params_.fixed_geo_reference = georef;

    if (state_.time_to_kf_id.empty()) {
      // No keyframes yet (the usual case: the front end loads the map before
      // any fusion starts). Rebuild the graph from scratch so T_enu_to_map is
      // created with the tight prior directly. state_.clear() first: the
      // pending newValues/newFactors still hold the symbol_T_enu_to_map entry
      // from the initialize()-time reinitialize_gtsam_locked(), and inserting
      // that key a second time would throw on the next solve.
      state_.clear();
      reinitialize_gtsam_locked();
      reset_sensor_anchors_locked();
      rebuilt = true;
    } else {
      // The central map is shared, persistent state and must survive this call,
      // so do not wipe it. Pin the EXISTING T_enu_to_map variable with an extra
      // prior instead: that removes the gauge freedom without discarding the
      // keyframes.
      gtsam::Pose3 enu2map;
      gtsam::Matrix6 enu2map_cov;
      mrpt::gtsam_wrappers::to_gtsam_se3_cov6(georef.T_enu_to_map, enu2map, enu2map_cov);
      state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);

      state_.geo_reference = georef;
      state_.last_estimated_frames[REFERENCE_FRAME_ID] = georef.T_enu_to_map;

      MRPT_LOG_WARN_FMT(
        "[set_geo_reference] Called with %zu keyframes already in the map: pinning the existing "
        "T_enu_to_map with an additional prior instead of rebuilding.",
        state_.time_to_kf_id.size());
    }
  }

  // Outside the lock: the optimizer takes stateMutex_ itself.
  if (!rebuilt) {
    notify_optimizer();
  }
}

// ---------------------------------------------------------------------------
// Keyframe-neighbor helpers
// ---------------------------------------------------------------------------
Mapper::pair_nearby_frame_iterators_t Mapper::find_before_after(
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

std::optional<KeyFrameID> Mapper::pick_closest(
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

std::optional<KeyFrameID> Mapper::find_nearest_kf_locked(const mrpt::Clock::time_point & t) const
{
  return pick_closest(find_before_after(t, true), t);
}

bool Mapper::sensor_kf_creation_allowed() const
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

}  // namespace mola::mapper
