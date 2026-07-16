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
#include <unordered_map>
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
  // Allow raising the background LC engine verbosity independently of the
  // mapper's own logger (e.g. to inspect per-candidate ICP/global-registration
  // details), via env var. Defaults to the mapper's level.
  if (const char * v = ::getenv("MOLA_VERBOSITY_LOOP_CLOSURE"); v != nullptr) {
    engine->setMinLoggingLevel(
      mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(v));
  } else {
    engine->setMinLoggingLevel(this->getMinLoggingLevel());
  }
  const auto cfg = mola::load_yaml_file(params_.loop_closure_pipeline_file);
  engine->initialize(cfg);
  lc_engine_ = engine;

  lc_scan_.reset();

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

std::size_t Mapper::run_loop_closure_scan(bool forceFullScan)
{
  if (!lc_engine_) {
    return 0;
  }

  // 1) Snapshot the central map under the state lock (cheap gate first, to
  //    avoid copying observations when the map barely grew). The finalize pass
  //    re-examines the same (complete) map every round, so it bypasses the
  //    min-new-keyframes gate.
  mrpt::maps::CSimpleMap snapshot;
  std::vector<KeyFrameID> frameIds;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    const std::size_t kfCount = state_.time_to_kf_id.size();
    if (
      !forceFullScan &&
      kfCount < lc_scan_.kf_count_at_last_scan + params_.loop_closure_min_new_keyframes) {
      return 0;  // not enough new keyframes yet
    }
    snapshot = state_.as_simple_map(&frameIds);
    lc_scan_.kf_count_at_last_scan = kfCount;
  }
  if (snapshot.size() < 2) {
    return 0;
  }

  // 2) Decide incremental vs full scan.
  bool fullScan = forceFullScan || !params_.loop_closure_incremental;
  if (
    params_.loop_closure_incremental && params_.loop_closure_full_scan_every_n > 0 &&
    lc_scan_.incremental_scans_since_full >= params_.loop_closure_full_scan_every_n) {
    fullScan = true;
  }

  mola::LoopClosureAnalyzeOptions opts;
  if (fullScan) {
    lc_scan_.incremental_scans_since_full = 0;
  } else {
    opts.first_new_keyframe = static_cast<uint32_t>(lc_scan_.snapshot_size_at_last_scan);
    lc_scan_.incremental_scans_since_full++;
  }

  // Abort promptly on shutdown.
  opts.should_abort = [this] { return lc_should_exit_.load(); };

  // Exclude already-closed pairs from candidate selection so each scan spends
  // its budget on new loops instead of re-proposing the same ones. merged_pairs
  // holds keyframe ids; map them to snapshot indices for this scan.
  {
    std::unordered_map<KeyFrameID, uint32_t> kfIdToIndex;
    kfIdToIndex.reserve(frameIds.size());
    for (uint32_t i = 0; i < frameIds.size(); i++) {
      kfIdToIndex[frameIds[i]] = i;
    }
    for (const auto & [a, b] : lc_scan_.merged_pairs) {
      const auto ia = kfIdToIndex.find(a);
      const auto ib = kfIdToIndex.find(b);
      if (ia != kfIdToIndex.end() && ib != kfIdToIndex.end()) {
        opts.exclude_pairs.insert(std::minmax(ia->second, ib->second));
      }
    }
  }

  // 3) Stream accepted edges into the graph as soon as they are found, mapping
  //    snapshot frame indices back to keyframe ids. The heavy ICP runs here
  //    OFF the state lock; only each merge briefly re-takes it. Count only
  //    edges actually added (a duplicate merge adds nothing).
  std::size_t merged = 0;
  opts.on_edge_found = [&](const mola::ProposedLoopEdge & e) {
    if (e.from >= frameIds.size() || e.to >= frameIds.size()) {
      return;
    }
    const KeyFrameID kfFrom = frameIds[e.from];
    const KeyFrameID kfTo = frameIds[e.to];
    bool added = false;
    {
      auto lck = mrpt::lockHelper(stateMutex_);
      added = merge_loop_closure_edge_locked(kfFrom, kfTo, e.relative_pose);
    }
    if (added) {
      merged++;
    }
  };

  lc_engine_->analyze(snapshot, opts);

  lc_scan_.snapshot_size_at_last_scan = snapshot.size();

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

void Mapper::finalize_loop_closures()
{
  if (!params_.loop_closure_enabled || !lc_engine_) {
    return;
  }
  if (params_.loop_closure_finalize_rounds == 0) {
    return;
  }

  // Stop the background LC thread (so we own the engine) but keep the engine
  // alive for the synchronous batch rounds below.
  {
    auto lk = mrpt::lockHelper(lc_wakeup_mutex_);
    lc_should_exit_.store(true);
  }
  lc_wakeup_cv_.notify_all();
  if (lc_thread_.joinable()) {
    lc_thread_.join();
  }
  lc_should_exit_.store(false);

  // Also stop the optimizer thread: the batch rounds re-optimize synchronously,
  // so a concurrently-running optimizer thread would race optimize_and_refresh().
  stop_optimizer_thread();

  MRPT_LOG_INFO_STREAM(
    "[loop_closure] finalize: up to " << params_.loop_closure_finalize_rounds
                                      << " batch full-scan rounds");

  std::size_t total = 0;
  for (uint32_t round = 0; round < params_.loop_closure_finalize_rounds; round++) {
    const std::size_t merged = run_loop_closure_scan(/*forceFullScan=*/true);
    if (merged == 0) {
      break;  // converged: no new loops this round
    }
    total += merged;
    // Re-optimize synchronously so the next round sees the reduced drift, which
    // brings further loop candidates within the ICP convergence basin.
    optimize_and_refresh();
    MRPT_LOG_INFO_STREAM(
      "[loop_closure] finalize round " << (round + 1) << ": merged " << merged << " edge(s) (total "
                                       << total << ")");
  }

  MRPT_LOG_INFO_STREAM("[loop_closure] finalize done: " << total << " edge(s) added");
}

bool Mapper::merge_loop_closure_edge_locked(
  KeyFrameID from, KeyFrameID to, const mrpt::poses::CPose3DPDFGaussian & relPose)
{
  if (from == to) {
    return false;
  }

  // Skip pairs already closed in a previous scan: a periodic full scan
  // re-proposes existing loops, and adding a second BetweenFactor for the same
  // pair would over-weight the constraint and grow the graph without bound.
  const std::pair<KeyFrameID, KeyFrameID> pairKey = std::minmax(from, to);
  if (!lc_scan_.merged_pairs.insert(pairKey).second) {
    return false;
  }

  // Convert the proposed relative pose + covariance to the GTSAM Pose3 tangent
  // (full 6x6, Jacobian-transformed and reordered) via the shared wrapper. A
  // small diagonal floor keeps the noise model non-degenerate for a near-perfect
  // ICP.
  mrpt::poses::CPose3DPDFGaussian pdf = relPose;
  pdf.cov.asEigen().diagonal().array() += 1e-8;

  gtsam::Pose3 relMean;
  gtsam::Matrix6 relCov;
  mrpt::gtsam_wrappers::to_gtsam_se3_cov6(pdf, relMean, relCov);

  // Robust kernel so a single spurious loop cannot deform the map (the plan's
  // GNC-in-parallel is a later refinement; a Huber M-estimator is the v1 guard).
  auto base = gtsam::noiseModel::Gaussian::Covariance(relCov);
  auto robust = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(params_.loop_closure_edge_robust_param), base);

  state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    T(from), T(to), relMean, robust);

  state_.add_kf_connectivity(from, to);
  return true;
}

}  // namespace mola::mapper
