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
 * @file   Mapper_KeyframeIngestion.cpp
 * @brief  SharedKeyframeMap sink: central-map keyframe ingestion from front
 *         ends (LIO/VIO).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Implements the "consecutive-frame edge" drift-fix design: the first request 
 * from a given source anchors F(source) with a tight
 * Between(F(source), T(kf)) factor; every later request from that same
 * source instead adds a tight Between(T(prev_kf), T(kf)) factor using the
 * front end's own *relative* motion since its last request. This preserves
 * local geometry without re-asserting the source's accumulated absolute
 * drift onto {map}, so IMU gravity, GNSS and loop-closure factors layered on
 * top of the keyframe poses remain free to override global z/tilt drift.
 *
 * Both factors always use OUR OWN configured noise
 * (keyframe_ingestion_sigma_lin / keyframe_ingestion_sigma_ang_deg, scaled by
 * the request's quality), never the request's own pose_in_source covariance:
 * real front ends can report wildly inconsistent absolute covariances (e.g. a
 * relocalization seed pinned at ~1e-12 m^2), which ill-conditions the graph
 * and was observed to throw gtsam::IndeterminantLinearSystemException in
 * practice when trusted directly.
 */

#include <gtsam/slam/BetweenFactor.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include "GtsamData.h"
#include "factor_builders.h"

namespace mola::mapper
{

std::optional<SharedKeyframeMap::KeyFrameID> Mapper::requestInsertKeyframe(
  const SharedKeyframeMap::KeyframeInsertRequest & req)
{
  const ProfilerEntry tle(profiler_, "requestInsertKeyframe");
  KeyFrameID kfId = 0;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    kfId = request_insert_keyframe_locked(req);
  }
  notify_optimizer();
  return kfId;
}

KeyFrameID Mapper::request_insert_keyframe_locked(
  const SharedKeyframeMap::KeyframeInsertRequest & req)
{
  ASSERTMSG_(!req.source_frame_id.empty(), "KeyframeInsertRequest::source_frame_id is empty.");

  // Signal that a SharedKeyframeMap producer is active. In Auto mode this
  // flips the behavior of sensor paths (fuse_pose/fuse_imu/fuse_odometry/
  // fuse_gnss) from legacy creation to SharedMapOnly: the sparse KF backbone
  // produced here becomes the sole creator of GTSAM keyframe variables, while
  // dense sensor data (e.g. LIO fuse_pose at ~10 Hz) only updates predictors.
  shared_kf_producer_active_ = true;

  const auto frameIdx = add_or_get_odom_frame_id_locked(req.source_frame_id);
  const auto kfId = create_or_get_keyframe_by_timestamp_locked(req.timestamp);

  // Merge the front end's raw observations into this keyframe's storage:
  auto & kfObs = state_.keyframe_observations[kfId];
  for (const auto & obs : req.observations) {
    kfObs.insert(obs);
  }

  // Feed this keyframe's absolute odometry pose (mean + covariance) into the
  // SAME single consecutive relative-pose-edge chain the dense fuse_pose() path
  // uses, so both sources contribute to ONE consistent backbone (no per-source
  // chains that skip each other's keyframes and create conflicting loop edges).
  // See Mapper::link_into_odometry_chain_locked(). The request covariance now
  // shapes the edge noise (anisotropic, per mola_sm_loop_closure), so z/tilt
  // stay soft for IMU-gravity / GNSS leveling. Pathological front-end values
  // (e.g. a near-zero relocalization-seed covariance) are bounded by the
  // odometry_edge_min_sigma_* floor in add_odom_chain_edge_locked().
  link_into_odometry_chain_locked(kfId, req.pose_in_source, frameIdx);

  // Phase B.1: wheel relative-pose edge between the previous sparse KF and
  // this one. Emits Between(T(prev_shared_kf), T(kfId)) from the net wheel
  // motion accumulated in last_wheels_odometry_ since the previous
  // requestInsertKeyframe() call. Fires whenever wheel data has been received;
  // the odom-chain edge from link_into_odometry_chain_locked already links
  // these KFs via LIO relative motion -- this adds a complementary wheel
  // constraint (planar motion model, looser sigma) at no extra variable cost.
  if (
    prev_shared_kf_id_.has_value() && prev_shared_kf_id_ != kfId &&
    last_wheels_odometry_.has_value() && wheel_odom_at_prev_shared_kf_.has_value()) {
    const auto increment = *last_wheels_odometry_ - *wheel_odom_at_prev_shared_kf_;

    mrpt::obs::CActionRobotMovement2D odoAct;
    odoAct.motionModelConfiguration.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
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
      T(*prev_shared_kf_id_), T(kfId), rel, gtsam::noiseModel::Gaussian::Covariance(relCov));
    state_.add_kf_connectivity(*prev_shared_kf_id_, kfId);

    MRPT_LOG_THROTTLE_DEBUG_FMT(
      5.0, "[SharedKeyframeMap] Added wheel edge KF#%llu→#%llu (Δx=%.3f m, Δy=%.3f m, Δφ=%.2f°)",
      static_cast<unsigned long long>(*prev_shared_kf_id_), static_cast<unsigned long long>(kfId),
      increment.x(), increment.y(), mrpt::RAD2DEG(increment.phi()));
  }
  // Update sparse-KF wheel anchor for the next interval.
  prev_shared_kf_id_ = kfId;
  wheel_odom_at_prev_shared_kf_ = last_wheels_odometry_;  // nullopt if no wheel sensor

  MRPT_LOG_THROTTLE_INFO_FMT(
    2.0, "[SharedKeyframeMap] Inserted keyframe #%llu from source '%s' (central map keyframes=%zu)",
    static_cast<unsigned long long>(kfId), req.source_frame_id.c_str(),
    state_.time_to_kf_id.size());

  return kfId;
}

}  // namespace mola::mapper
