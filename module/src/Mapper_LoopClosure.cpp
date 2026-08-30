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

#include <algorithm>
#include <chrono>
#include <exception>
#include <mutex>
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
  auto lcVerbosity = this->getMinLoggingLevel();
  if (const char * v = ::getenv("MOLA_VERBOSITY_LOOP_CLOSURE"); v != nullptr) {
    try {
      lcVerbosity = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(v);
    } catch (const std::exception & e) {
      MRPT_LOG_WARN_STREAM(
        "[loop_closure] ignoring invalid MOLA_VERBOSITY_LOOP_CLOSURE='" << v << "': " << e.what());
    }
  }
  engine->setMinLoggingLevel(lcVerbosity);
  const auto cfg = mola::load_yaml_file(params_.loop_closure_pipeline_file);
  engine->initialize(cfg);
  lc_engine_ = engine;

  lc_scan_.reset();
  lc_ui_.reset();

  lc_should_exit_.store(false);

  // With the thread disabled the engine above is still created and configured:
  // the caller drives each scan itself (run_loop_closure_scan_now()) and the
  // end-of-run finalize pass, which is synchronous anyway, still has an engine
  // to work with. Only the wall-clock-paced scan loop is skipped.
  if (!params_.enable_loop_closure_thread) {
    MRPT_LOG_INFO_STREAM(
      "[loop_closure] engine ready (caller-driven scans), pipeline='"
      << params_.loop_closure_pipeline_file << "'");
    return;
  }

  lc_thread_ = std::thread(&Mapper::loop_closure_thread_loop, this);

  MRPT_LOG_INFO_STREAM(
    "[loop_closure] started, pipeline='" << params_.loop_closure_pipeline_file << "'");
}

std::size_t Mapper::run_loop_closure_scan_now(bool forceFullScan)
{
  return run_loop_closure_scan(forceFullScan);
}

void Mapper::stop_loop_closure_thread() { stop_loop_closure_thread_locked(/*resetEngine=*/true); }

void Mapper::request_loop_closure_scan()
{
  if (!lc_thread_.joinable()) {
    return;  // LC thread not running
  }
  {
    auto lk = mrpt::lockHelper(lc_wakeup_mutex_);
    lc_force_scan_.store(true);
  }
  lc_wakeup_cv_.notify_all();
}

void Mapper::stop_loop_closure_thread_locked(bool resetEngine)
{
  // The thread is optional (enable_loop_closure_thread), but the engine is
  // not: releasing it must not depend on a thread having been spawned, or a
  // caller-driven configuration would keep a stale engine across initialize().
  if (lc_thread_.joinable()) {
    {
      auto lk = mrpt::lockHelper(lc_wakeup_mutex_);
      lc_should_exit_.store(true);
    }
    lc_wakeup_cv_.notify_all();
    lc_thread_.join();
  }
  if (resetEngine) {
    lc_engine_.reset();
  }
}

void Mapper::loop_closure_thread_loop()
{
  const auto period =
    std::chrono::duration<double>(std::max(0.1, params_.loop_closure_check_period_sec));

  while (!lc_should_exit_.load()) {
    {
      std::unique_lock<std::mutex> lk(lc_wakeup_mutex_);
      lc_wakeup_cv_.wait_for(
        lk, period, [this] { return lc_should_exit_.load() || lc_force_scan_.load(); });
    }
    if (lc_should_exit_.load()) {
      break;
    }

    // Mark busy BEFORE consuming the request, so a waiter can never sample the
    // gap between the two and conclude the thread is idle -- see lc_busy_.
    lc_busy_.store(true);

    // A manual request forces one full scan (bypassing the min-new-keyframes
    // gate); the periodic wake-up runs the normal incremental/full logic.
    const bool forceFull = lc_force_scan_.exchange(false);

    try {
      run_loop_closure_scan(forceFull);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("[loop_closure] scan failed: " << e.what());
    }

    // Clear and announce even if the scan threw: a waiter must not be stranded
    // by a failed scan.
    {
      auto lk = mrpt::lockHelper(lc_idle_mutex_);
      lc_busy_.store(false);
    }
    lc_idle_cv_.notify_all();
  }

  // Leaving the loop (shutdown) is also "no longer busy": a waiter blocked here
  // while the thread is asked to stop would otherwise wait out its whole
  // timeout for a thread that is never going to run again.
  {
    auto lk = mrpt::lockHelper(lc_idle_mutex_);
    lc_busy_.store(false);
  }
  lc_idle_cv_.notify_all();
}

bool Mapper::wait_for_loop_closure_idle(double timeoutSeconds)
{
  if (!lc_thread_.joinable()) {
    return true;  // no thread to wait for; the caller owns the schedule
  }

  std::unique_lock<std::mutex> lk(lc_idle_mutex_);
  const auto isIdle = [this] { return !lc_force_scan_.load() && !lc_busy_.load(); };

  if (timeoutSeconds <= 0) {
    lc_idle_cv_.wait(lk, isIdle);
    return true;
  }
  return lc_idle_cv_.wait_for(lk, std::chrono::duration<double>(timeoutSeconds), isIdle);
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

  register_lc_metrics_if_needed();
  const auto scanT0 = std::chrono::steady_clock::now();
  lc_ui_.scan_in_progress.store(true);
  lc_ui_.cur_total.store(0);
  lc_ui_.cur_done.store(0);
  // Clear the in-progress flag on any exit path (including if analyze() throws),
  // so a failed scan never leaves the UI stuck showing "scan in progress".
  struct InProgressGuard
  {
    std::atomic<bool> & flag;
    ~InProgressGuard() { flag.store(false); }
  } inProgressGuard{lc_ui_.scan_in_progress};

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

  // Live per-candidate progress: drives the GUI "pending this scan" counter
  // (queue depth = total - done) and the queue-depth metric plot.
  mola::LoopClosureAnalyzeStats stats;
  opts.out_stats = &stats;
  opts.on_progress = [this](std::size_t done, std::size_t total) {
    lc_ui_.cur_total.store(total);
    lc_ui_.cur_done.store(done);
#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
    if (metric_lc_queue_depth_) {
      metric_lc_queue_depth_->push(static_cast<double>(total - done));
    }
#endif
  };

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

  // 3) COLLECT accepted edges, then merge them in a canonical order once the
  //    scan is over.
  //
  //    Not merged from the callback, which is where they used to go. The
  //    detector evaluates candidates on its own worker pool, sized from
  //    hardware_concurrency(), so on_edge_found fires from whichever thread
  //    finished first -- an order that varies run to run. That order is
  //    observable: merge_loop_closure_edge_locked() drops a pair already
  //    closed, so which of two overlapping proposals wins depends on it, and
  //    every later scan then starts from a different graph and a different
  //    exclude_pairs set. Two identical offline runs of KITTI-00 diverged for
  //    exactly this reason. Sorting by keyframe id before merging removes the
  //    dependence on thread scheduling without changing what the detector does.
  //
  //    Nothing is lost by waiting: notify_optimizer() is only called after
  //    analyze() returns anyway, so no merge was ever visible to the solver
  //    mid-scan. It also removes a data race -- `merged` was a plain counter
  //    incremented from those worker threads.
  struct AcceptedEdge
  {
    KeyFrameID from;
    KeyFrameID to;
    mrpt::poses::CPose3DPDFGaussian relative_pose;
    double quality;
  };
  std::vector<AcceptedEdge> accepted;
  std::mutex acceptedMutex;

  opts.on_edge_found = [&](const mola::ProposedLoopEdge & e) {
    if (e.from >= frameIds.size() || e.to >= frameIds.size()) {
      return;
    }
    auto lck = mrpt::lockHelper(acceptedMutex);
    accepted.push_back({frameIds[e.from], frameIds[e.to], e.relative_pose, e.quality});
  };

  lc_engine_->analyze(snapshot, opts);

  std::sort(accepted.begin(), accepted.end(), [](const AcceptedEdge & a, const AcceptedEdge & b) {
    return std::minmax(a.from, a.to) < std::minmax(b.from, b.to);
  });

  std::size_t merged = 0;
  for (const auto & e : accepted) {
    bool added = false;
    {
      auto lck = mrpt::lockHelper(stateMutex_);
      added = merge_loop_closure_edge_locked(e.from, e.to, e.relative_pose);
    }
    if (!added) {
      continue;
    }
    merged++;
    lc_ui_.loops_accepted.fetch_add(1);
#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
    if (metric_lc_edge_goodness_) {
      metric_lc_edge_goodness_->push(100.0 * e.quality);
    }
#endif
  }

  lc_scan_.snapshot_size_at_last_scan = snapshot.size();

  // Publish per-scan UI counters + metrics.
  const double scanSeconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - scanT0).count();
  lc_ui_.candidates_checked.fetch_add(stats.candidates_evaluated);
  lc_ui_.scans_completed.fetch_add(1);
  lc_ui_.last_scan_accepted.store(merged);
  lc_ui_.last_scan_seconds.store(scanSeconds);
  lc_ui_.last_scan_full.store(fullScan);
  lc_ui_.cur_done.store(lc_ui_.cur_total.load());
#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
  if (metric_lc_loops_total_) {
    metric_lc_loops_total_->push(static_cast<double>(lc_ui_.loops_accepted.load()));
    metric_lc_candidates_per_scan_->push(static_cast<double>(stats.candidates_generated));
    metric_lc_scan_time_ms_->push(1000.0 * scanSeconds);
  }
#endif

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

void Mapper::register_lc_metrics_if_needed()
{
#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
  if (lc_metrics_registered_ || !visualizer_) {
    return;
  }
  metric_lc_loops_total_ = visualizer_->register_metric("mapper/lc_loops_total", "loops");
  metric_lc_queue_depth_ = visualizer_->register_metric("mapper/lc_queue_depth", "pending");
  metric_lc_candidates_per_scan_ =
    visualizer_->register_metric("mapper/lc_candidates_per_scan", "cands");
  metric_lc_scan_time_ms_ = visualizer_->register_metric("mapper/lc_scan_time_ms", "ms");
  metric_lc_edge_goodness_ = visualizer_->register_metric("mapper/lc_edge_goodness", "%");
  lc_metrics_registered_ = true;
#endif
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
  stop_loop_closure_thread_locked(/*resetEngine=*/false);
  lc_should_exit_.store(false);

  // Also stop the optimizer thread: the batch rounds re-optimize synchronously,
  // so a concurrently-running optimizer thread would race optimize_and_refresh().
  stop_optimizer_thread();

  MRPT_LOG_INFO_STREAM(
    "[loop_closure] finalize: up to " << params_.loop_closure_finalize_rounds
                                      << " batch full-scan rounds");

  lc_ui_.finalize_active.store(true);
  lc_ui_.finalize_rounds_total.store(params_.loop_closure_finalize_rounds);

  const auto t0 = std::chrono::steady_clock::now();

  std::size_t total = 0;
  for (uint32_t round = 0; round < params_.loop_closure_finalize_rounds; round++) {
    lc_ui_.finalize_round.store(round + 1);
    if (params_.loop_closure_finalize_max_seconds > 0) {
      const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed.count() >= params_.loop_closure_finalize_max_seconds) {
        MRPT_LOG_WARN_STREAM(
          "[loop_closure] finalize: time budget of " << params_.loop_closure_finalize_max_seconds
                                                     << " s reached after " << round
                                                     << " round(s), stopping");
        break;
      }
    }
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

  lc_ui_.finalize_active.store(false);

  MRPT_LOG_INFO_STREAM("[loop_closure] finalize done: " << total << " edge(s) added");
}

std::vector<std::pair<Mapper::ChildLoggerName, mrpt::system::COutputLogger *>>
Mapper::child_loggers() const
{
  if (!lc_engine_) {
    return {};
  }
  return {{"loop_closure", lc_engine_.get()}};
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
