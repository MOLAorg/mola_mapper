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
 * @file   Mapper_LoopClosure.cpp
 * @brief  Background loop-closure thread: runs the mola_sm_loop_closure
 *         detector on a map snapshot and merges accepted edges into the graph.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_mapper/Mapper.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <chrono>
#include <exception>
#include <utility>
#include <vector>

#include "GtsamData.h"
#include "factor_builders.h"

namespace mola::mapper
{
void Mapper::start_loop_closure_thread()
{
  if (!params_.loop_closure_enabled) {
    return;
  }
  ASSERTMSG_(
    !params_.loop_closure_pipeline_file.empty(),
    "loop_closure_enabled is true but loop_closure_pipeline_file is empty.");

  // Create and configure the detector engine (F2F). We only ever call its
  // detector-only analyze() from the LC thread; process() is not used here.
  auto engine = std::make_shared<mola::FrameToFrameLoopClosure>();
  const auto cfg = mola::load_yaml_file(params_.loop_closure_pipeline_file);
  engine->initialize(cfg);
  lc_engine_ = engine;

  lc_kf_count_at_last_scan_ = 0;
  lc_snapshot_size_at_last_scan_ = 0;
  lc_incremental_scans_since_full_ = 0;
  lc_merged_pairs_.clear();

  lc_should_exit_.store(false);
  lc_thread_ = std::thread(&Mapper::loop_closure_thread_loop, this);

  MRPT_LOG_INFO_STREAM(
    "[loop_closure] started, pipeline='" << params_.loop_closure_pipeline_file << "'");
}

void Mapper::stop_loop_closure_thread()
{
  if (!lc_thread_.joinable()) {
    return;
  }
  {
    auto lk = mrpt::lockHelper(lc_wakeup_mutex_);
    lc_should_exit_.store(true);
  }
  lc_wakeup_cv_.notify_all();
  lc_thread_.join();
  lc_engine_.reset();
}

void Mapper::loop_closure_thread_loop()
{
  const auto period =
    std::chrono::duration<double>(std::max(0.1, params_.loop_closure_check_period_sec));

  while (!lc_should_exit_.load()) {
    {
      std::unique_lock<std::mutex> lk(lc_wakeup_mutex_);
      lc_wakeup_cv_.wait_for(lk, period, [this] { return lc_should_exit_.load(); });
    }
    if (lc_should_exit_.load()) {
      break;
    }

    try {
      run_loop_closure_scan();
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("[loop_closure] scan failed: " << e.what());
    }
  }
}

std::size_t Mapper::run_loop_closure_scan()
{
  if (!lc_engine_) {
    return 0;
  }

  // 1) Snapshot the central map under the state lock (cheap gate first, to
  //    avoid copying observations when the map barely grew).
  mrpt::maps::CSimpleMap snapshot;
  std::vector<KeyFrameID> frameIds;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    const std::size_t kfCount = state_.time_to_kf_id.size();
    if (kfCount < lc_kf_count_at_last_scan_ + params_.loop_closure_min_new_keyframes) {
      return 0;  // not enough new keyframes yet
    }
    snapshot = state_.as_simple_map(&frameIds);
    lc_kf_count_at_last_scan_ = kfCount;
  }
  if (snapshot.size() < 2) {
    return 0;
  }

  // 2) Decide incremental vs full scan.
  bool fullScan = !params_.loop_closure_incremental;
  if (
    params_.loop_closure_incremental && params_.loop_closure_full_scan_every_n > 0 &&
    lc_incremental_scans_since_full_ >= params_.loop_closure_full_scan_every_n) {
    fullScan = true;
  }

  mola::LoopClosureAnalyzeOptions opts;
  if (fullScan) {
    lc_incremental_scans_since_full_ = 0;
  } else {
    opts.first_new_keyframe = static_cast<uint32_t>(lc_snapshot_size_at_last_scan_);
    lc_incremental_scans_since_full_++;
  }

  // Abort promptly on shutdown.
  opts.should_abort = [this] { return lc_should_exit_.load(); };

  // 3) Stream accepted edges into the graph as soon as they are found, mapping
  //    snapshot frame indices back to keyframe ids. The heavy ICP runs here
  //    OFF the state lock; only each merge briefly re-takes it.
  std::size_t merged = 0;
  opts.on_edge_found = [&](const mola::ProposedLoopEdge & e) {
    if (e.from >= frameIds.size() || e.to >= frameIds.size()) {
      return;
    }
    const KeyFrameID kfFrom = frameIds[e.from];
    const KeyFrameID kfTo = frameIds[e.to];
    {
      auto lck = mrpt::lockHelper(stateMutex_);
      merge_loop_closure_edge_locked(kfFrom, kfTo, e.relative_pose);
    }
    merged++;
  };

  lc_engine_->analyze(snapshot, opts);

  lc_snapshot_size_at_last_scan_ = snapshot.size();

  if (merged > 0) {
    // Hand the new edges to the optimizer (background thread, or the next
    // synchronous query flush when the optimizer thread is disabled).
    notify_optimizer();
    MRPT_LOG_INFO_STREAM(
      "[loop_closure] merged " << merged << " edge(s) from a "
                               << (fullScan ? "full" : "incremental") << " scan of "
                               << snapshot.size() << " keyframes");
  }

  return merged;
}

void Mapper::merge_loop_closure_edge_locked(
  KeyFrameID from, KeyFrameID to, const mrpt::poses::CPose3DPDFGaussian & relPose)
{
  if (from == to) {
    return;
  }

  // Skip pairs already closed in a previous scan: a periodic full scan
  // re-proposes existing loops, and adding a second BetweenFactor for the same
  // pair would over-weight the constraint and grow the graph without bound.
  const std::pair<KeyFrameID, KeyFrameID> pairKey = std::minmax(from, to);
  if (!lc_merged_pairs_.insert(pairKey).second) {
    return;
  }

  // Per-DOF sigmas from the proposed relative-pose covariance. MRPT covariance
  // order is [x, y, z, yaw, pitch, roll]; GTSAM Pose3 tangent order is
  // (roll, pitch, yaw, x, y, z). A small floor keeps the noise model
  // non-degenerate for a near-perfect ICP.
  const gtsam::Vector6 sigXYZYPR =
    relPose.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();

  gtsam::Vector6 sigmas;
  sigmas << sigXYZYPR[5], sigXYZYPR[4], sigXYZYPR[3], sigXYZYPR[0], sigXYZYPR[1], sigXYZYPR[2];
  sigmas = sigmas.cwiseMax(1e-4);

  // Robust kernel so a single spurious loop cannot deform the map (the plan's
  // GNC-in-parallel is a later refinement; a Huber M-estimator is the v1 guard).
  auto base = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  auto robust = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(params_.loop_closure_edge_robust_param), base);

  state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    T(from), T(to), mrpt::gtsam_wrappers::toPose3(relPose.mean), robust);

  state_.add_kf_connectivity(from, to);
}

}  // namespace mola::mapper
