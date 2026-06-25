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
 * @file   Mapper3D_KeyframeIngestion.cpp
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
#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include "GtsamData.h"
#include "factor_builders.h"

namespace mola::mapper_3d
{

std::optional<SharedKeyframeMap::KeyFrameID> Mapper3D::requestInsertKeyframe(
  const SharedKeyframeMap::KeyframeInsertRequest & req)
{
  KeyFrameID kfId = 0;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    kfId = request_insert_keyframe_locked(req);
  }
  notify_optimizer();
  return kfId;
}

KeyFrameID Mapper3D::request_insert_keyframe_locked(
  const SharedKeyframeMap::KeyframeInsertRequest & req)
{
  ASSERTMSG_(!req.source_frame_id.empty(), "KeyframeInsertRequest::source_frame_id is empty.");

  const auto frameIdx = add_or_get_odom_frame_id_locked(req.source_frame_id);
  const auto kfId = create_or_get_keyframe_by_timestamp_locked(req.timestamp);

  // Merge the front end's raw observations into this keyframe's storage:
  auto & kfObs = state_.keyframe_observations[kfId];
  for (const auto & obs : req.observations) {
    kfObs.insert(obs);
  }

  const double linSigma = params_.keyframe_ingestion_sigma_lin / std::max(req.quality, 1e-3);
  const double angSigma =
    mrpt::DEG2RAD(params_.keyframe_ingestion_sigma_ang_deg) / std::max(req.quality, 1e-3);
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector6{angSigma, angSigma, angSigma, linSigma, linSigma, linSigma});

  const auto sourceState = keyframe_ingestion_state_by_source_.find(req.source_frame_id);

  if (sourceState == keyframe_ingestion_state_by_source_.end()) {
    // First request from this source: anchor F(source) to this keyframe with
    // a tight Between() factor (does NOT repeat for later requests, so the
    // source's own accumulated absolute drift never re-enters the graph).
    state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      symbol_T_map_to_odom_i_base + frameIdx, T(kfId),
      mrpt::gtsam_wrappers::toPose3(req.pose_in_source.mean), noise);
  } else if (sourceState->second.last_kf_id != kfId) {
    // Later request, and it landed on a different keyframe than the previous
    // one from this source (it may coincide with the previous keyframe when
    // the out-of-order guard snapped it): tight consecutive relative-pose
    // factor directly between keyframe poses, using the front end's own
    // relative motion (frame-invariant, so the source-frame composition
    // cancels out).
    const mrpt::poses::CPose3D relativeMotion =
      req.pose_in_source.mean - sourceState->second.last_pose_in_source;

    state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      T(sourceState->second.last_kf_id), T(kfId), mrpt::gtsam_wrappers::toPose3(relativeMotion),
      noise);

    state_.add_kf_connectivity(sourceState->second.last_kf_id, kfId);
  }

  keyframe_ingestion_state_by_source_[req.source_frame_id] = {kfId, req.pose_in_source.mean};

  MRPT_LOG_THROTTLE_INFO_FMT(
    2.0, "[SharedKeyframeMap] Inserted keyframe #%llu from source '%s' (central map keyframes=%zu)",
    static_cast<unsigned long long>(kfId), req.source_frame_id.c_str(),
    state_.time_to_kf_id.size());

  return kfId;
}

}  // namespace mola::mapper_3d
