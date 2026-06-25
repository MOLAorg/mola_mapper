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
  const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp);

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
    // Reference frame is "map": a direct pose prior on the keyframe.
    state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      T(this_kf_id), pose_out, gtsam::noiseModel::Gaussian::Covariance(cov_out));
  } else {
    // Odometry frame: BetweenFactor(F(k), T(kf)) measures T_odom_k_to_base.
    state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
      gtsam::noiseModel::Gaussian::Covariance(cov_out));
  }
}

void Mapper3D::fuse_odometry(
  const mrpt::obs::CObservationOdometry & odom, const std::string & odomName)
{
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

  // High-rate decimation/merge: if this reading arrives too soon after the last
  // *processed* one, drop it WITHOUT advancing the anchor, so the next kept
  // reading fuses the full accumulated increment + accumulated covariance.
  if (params_.odometry_min_sample_period > 0 && last_wheels_odometry_stamp_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_wheels_odometry_stamp_, odom.timestamp);
    if (dt < params_.odometry_min_sample_period) {
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
    const auto kfId =
      create_or_get_keyframe_by_timestamp_locked(odom.timestamp, params_.sensor_keyframe_min_period);

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
  auto lck = mrpt::lockHelper(stateMutex_);

  // High-rate decimation: skip readings arriving too soon after the last
  // processed one (attitude/gravity are absolute, so dropping just lowers the
  // redundant-factor rate; no increment merging needed).
  if (params_.imu_min_sample_period > 0 && last_processed_imu_stamp_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_processed_imu_stamp_, imu.timestamp);
    if (dt < params_.imu_min_sample_period) {
      return;
    }
  }
  last_processed_imu_stamp_ = imu.timestamp;

  // In aggregation mode, attach to the shared bounded-rate keyframe clock (the
  // same one wheel odometry uses), so IMU does not spawn its own dense
  // keyframes either.
  const double imuKfTolerance = params_.aggregate_high_rate_into_edges
                                  ? params_.sensor_keyframe_min_period
                                  : params_.imu_nearby_keyframe_stamp_tolerance;
  const auto this_kf_id =
    create_or_get_keyframe_by_timestamp_locked(imu.timestamp, imuKfTolerance);

  const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

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
  }

  notify_optimizer();
}

void Mapper3D::fuse_gnss(const mrpt::obs::CObservationGPS & gps)
{
  auto lck = mrpt::lockHelper(stateMutex_);

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

  const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(
    gps.timestamp, params_.gnss_nearby_keyframe_stamp_tolerance);

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
  std::scoped_lock solveLock(solve_mutex_);

  auto & gd = *state_.gtsam;
  ASSERT_(gd.isam2.has_value());

  // ---- Phase A: grab pending work + snapshot (brief lock) ----
  gtsam::NonlinearFactorGraph localFactors;
  gtsam::Values localValues;
  std::vector<KeyFrameID> snapshotKfIds;
  std::vector<OdometryFrameID> snapshotFrameIds;
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
    gd.estimate = gd.isam2->calculateEstimate();
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
      if (!gd.estimate.exists(T(id))) {
        continue;
      }
      TmpKf t;
      t.pose = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(gd.estimate.at<gtsam::Pose3>(T(id))));
      const auto linV = gd.estimate.at<gtsam::Vector3>(V(id));
      const auto angV = gd.estimate.at<gtsam::Vector3>(W(id));
      t.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};
      if (params_.enforce_planar_motion) {
        enforce_planar_pose(t.pose);
        enforce_planar_twist(t.twist);
      }
      tmpStates.emplace(id, t);
    }

    // Marginal covariance ONLY for the latest keyframe (the predictor's anchor
    // in steady state): cheap, and keeps the query path iSAM2-free.
    if (haveKfs && gd.estimate.exists(T(latestKfId))) {
      latestPoseCov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(
        gtsam::Matrix6(gd.isam2->marginalCovariance(T(latestKfId))));
      mrpt::math::CMatrixDouble66 twCov;
      twCov.setZero();
      twCov.asEigen().block<3, 3>(0, 0) = gd.isam2->marginalCovariance(V(latestKfId));
      twCov.asEigen().block<3, 3>(3, 3) = gd.isam2->marginalCovariance(W(latestKfId));
      latestTwistCov = twCov;
    }

    for (const OdometryFrameID fid : snapshotFrameIds) {
      const auto Fpose = gd.estimate.at<gtsam::Pose3>(symbol_T_map_to_odom_i_base + fid);
      const auto Fcov = gd.isam2->marginalCovariance(symbol_T_map_to_odom_i_base + fid);
      mrpt::poses::CPose3DPDFGaussian pdf;
      pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(Fpose));
      pdf.cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(Fcov);
      tmpFrames.emplace(fid, pdf);
    }

    if (estimateGeoref) {
      const auto Te = gd.estimate.at<gtsam::Pose3>(symbol_T_enu_to_map);
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
        const bool converged =
          std::max(enuPosSigma, kfPosSigma) <= params_.convergence_max_position_sigma &&
          std::max(enuOriSigmaDeg, kfOriSigmaDeg) <= params_.convergence_max_orientation_sigma_deg;
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
    for (const auto & [fid, pdf] : tmpFrames) {
      state_.last_estimated_frames[fid] = pdf;
    }
    if (tmpEnu.has_value()) {
      state_.last_estimated_frames[REFERENCE_FRAME_ID] = *tmpEnu;
    }
    if (tmpGeoRef.has_value()) {
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

  if (!closestFrameIdx.has_value() || *closestFrameDt > params_.max_time_to_use_velocity_model) {
    return {};
  }

  // 3) Recover the closest state (in the reference/map frame) and extrapolate.
  NavState ret = get_latest_state_and_covariance(*closestFrameIdx);

  mrpt::math::CVectorFixed<double, 6> twistDt;
  twistDt[0] = ret.twist.vx;
  twistDt[1] = ret.twist.vy;
  twistDt[2] = ret.twist.vz;
  twistDt[3] = ret.twist.wx;
  twistDt[4] = ret.twist.wy;
  twistDt[5] = ret.twist.wz;
  twistDt *= closestFrameDtSigned;
  ret.pose.mean = ret.pose.mean + mrpt::poses::Lie::SE<3>::exp(twistDt);

  // Approximate uncertainty growth due to random walk:
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

  // 4) Convert the pose to the requested frame_id, if not the reference frame.
  if (frame_id != params_.reference_frame_name) {
    const auto it = state_.known_odom_frames.find_key(frame_id);
    if (it == state_.known_odom_frames.getDirectMap().end()) {
      MRPT_LOG_THROTTLE_WARN_FMT(
        5.0, "[estimated_navstate] Requested unknown odometry frame_id='%s'", frame_id.c_str());
      return {};
    }
    const auto requestedFrameIdx = it->second;
    const auto itFrame = state_.last_estimated_frames.find(requestedFrameIdx);
    if (itFrame == state_.last_estimated_frames.end()) {
      return {};
    }
    mrpt::poses::CPose3DPDFGaussianInf posePdfFrame_wrt_map_inf;
    posePdfFrame_wrt_map_inf.copyFrom(itFrame->second);
    ret.pose = ret.pose - posePdfFrame_wrt_map_inf;
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
  using Iterator = stamp_map_t::const_iterator;

  if (m.empty()) {
    return {m.end(), m.end()};
  }

  Iterator after = m.upper_bound(t);

  if (!allow_exact_match) {
    Iterator before = (after == m.begin()) ? m.end() : std::prev(after);
    if (before != m.end() && before->first == t) {
      Iterator element_before_match = (before == m.begin()) ? m.end() : std::prev(before);
      return {element_before_match, after};
    }
    return {before, after};
  }

  if (after == m.begin()) {
    return {m.end(), after};
  }
  Iterator before = std::prev(after);
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

}  // namespace mola::mapper_3d
