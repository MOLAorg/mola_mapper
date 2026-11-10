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
 * @brief  Background loop closure detection with the mola_sm_loop_closure engine.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/LoopConsistency.h>
#include <mola_mapper/LoopOdometryCheck.h>
#include <mola_mapper/Mapper.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

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

  // The engine is only used through its detector (analyze()); the global graph
  // is the mapper's own.
  auto engine = std::make_shared<mola::FrameToFrameLoopClosure>();
  auto verbosity = this->getMinLoggingLevel();
  if (const auto v = mrpt::get_env<std::string>("MOLA_VERBOSITY_LOOP_CLOSURE", ""); !v.empty()) {
    try {
      verbosity = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(v);
    } catch (const std::exception &) {
      MRPT_LOG_WARN_STREAM("Ignoring invalid MOLA_VERBOSITY_LOOP_CLOSURE='" << v << "'");
    }
  }
  engine->setMinLoggingLevel(verbosity);
  engine->initialize(mola::load_yaml_file(params_.loop_closure_pipeline_file));
  lc_engine_ = engine;

  if (!params_.enable_loop_closure_thread) {
    MRPT_LOG_INFO("Loop closure engine ready (scans driven by the caller)");
    return;
  }
  lc_should_exit_.store(false);
  lc_thread_running_.store(true);
  lc_thread_ = std::thread(&Mapper::loop_closure_thread_loop, this);
  MRPT_LOG_INFO_STREAM(
    "Loop closure thread started, pipeline: " << params_.loop_closure_pipeline_file);
}

void Mapper::stop_loop_closure_thread()
{
  lc_should_exit_.store(true);
  {
    auto lk = mrpt::lockHelper(lc_wakeup_mtx_);
  }
  lc_wakeup_cv_.notify_all();
  if (lc_thread_.joinable()) {
    lc_thread_.join();
  }
}

void Mapper::request_loop_closure_scan()
{
  {
    auto lk = mrpt::lockHelper(lc_wakeup_mtx_);
    lc_force_scan_.store(true);
  }
  lc_wakeup_cv_.notify_all();
}

void Mapper::loop_closure_thread_loop()
{
  const auto period =
    std::chrono::duration<double>(std::max(0.1, params_.loop_closure_check_period_sec));

  while (!lc_should_exit_.load()) {
    {
      std::unique_lock<std::mutex> lk(lc_wakeup_mtx_);
      lc_wakeup_cv_.wait_for(
        lk, period, [this] { return lc_should_exit_.load() || lc_force_scan_.load(); });
    }
    if (lc_should_exit_.load()) {
      break;
    }
    // Busy BEFORE consuming the request, so a waiter never samples the gap:
    lc_busy_.store(true);
    const bool forceFull = lc_force_scan_.exchange(false);
    try {
      run_loop_closure_scan(forceFull);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Loop closure scan failed: " << e.what());
    }
    {
      auto lk = mrpt::lockHelper(lc_idle_mtx_);
      lc_busy_.store(false);
    }
    lc_idle_cv_.notify_all();
  }
  {
    auto lk = mrpt::lockHelper(lc_idle_mtx_);
    lc_busy_.store(false);
    lc_thread_running_.store(false);
  }
  lc_idle_cv_.notify_all();
}

bool Mapper::wait_for_loop_closure_idle(double timeoutSeconds)
{
  std::unique_lock<std::mutex> lk(lc_idle_mtx_);
  // A request left pending by a thread that exited is never served: idle too.
  const auto isIdle = [this] {
    return !lc_thread_running_.load() || (!lc_force_scan_.load() && !lc_busy_.load());
  };
  if (timeoutSeconds <= 0) {
    lc_idle_cv_.wait(lk, isIdle);
    return true;
  }
  return lc_idle_cv_.wait_for(lk, std::chrono::duration<double>(timeoutSeconds), isIdle);
}

std::optional<LoopOdometryCheck> Mapper::check_against_odometry_locked(
  std::size_t first, std::size_t last, const mola::ProposedLoopEdge & e) const
{
  // Only within one front end chain: its poses are all in the same odometry frame.
  std::vector<mrpt::poses::CPose3D> chain;
  chain.reserve(last - first + 1);
  for (std::size_t i = first; i <= last; i++) {
    if (i > first && keyframes_[i].source_frame != keyframes_[i - 1].source_frame) {
      return std::nullopt;
    }
    chain.push_back(keyframes_[i].pose_in_source.mean);
  }
  return check_loop_against_odometry(
    chain, e.from - first, e.to - first, e.relative_pose.mean,
    params_.loop_closure_odometry_limits);
}

std::size_t Mapper::drop_inconsistent_loops_locked(std::size_t firstNew)
{
  OdometryChains odo;
  odo.pose_in_source.reserve(keyframes_.size());
  odo.chain_id.reserve(keyframes_.size());
  std::map<std::string, std::size_t> chainIds;
  for (const auto & kf : keyframes_) {
    odo.pose_in_source.push_back(kf.pose_in_source.mean);
    odo.chain_id.push_back(chainIds.try_emplace(kf.source_frame, chainIds.size()).first->second);
  }
  std::vector<LoopForConsistency> loops;
  loops.reserve(loops_.size());
  for (const auto & l : loops_) {
    LoopForConsistency c;
    c.from = l.from;
    c.to = l.to;
    c.relative_pose = l.relative_pose.mean;
    loops.push_back(c);
  }
  const auto bad = find_inconsistent_loops(odo, loops, firstNew, params_.loop_closure_consistency);
  if (bad.empty()) {
    return 0;
  }
  std::size_t droppedNew = 0;
  std::vector<bool> drop(loops_.size(), false);
  for (const auto k : bad) {
    drop[k] = true;
    const auto & l = loops_[k];
    const std::pair<std::size_t, std::size_t> key = {
      std::min(l.from, l.to), std::max(l.from, l.to)};
    loop_pairs_.erase(key);
    rejected_loop_pairs_.insert(key);
    if (k >= firstNew) {
      droppedNew++;
    }
    MRPT_LOG_INFO_FMT(
      "Loop %zu-%zu %s: inconsistent with most of its neighbor loops", key.first, key.second,
      k >= firstNew ? "rejected" : "dropped");
  }
  std::vector<GlobalGraphLoopEdge> kept;
  kept.reserve(loops_.size() - bad.size());
  for (std::size_t k = 0; k < loops_.size(); k++) {
    if (!drop[k]) {
      kept.push_back(loops_[k]);
    }
  }
  loops_ = std::move(kept);
  lc_rejected_by_consistency_ += bad.size();
  if (droppedNew < bad.size()) {
    // An accepted loop was dropped: the graph must be solved again.
    optimizer_new_data_.store(true);
  }
  return droppedNew;
}

std::size_t Mapper::run_loop_closure_scan_now(bool forceFullScan)
{
  return run_loop_closure_scan(forceFullScan);
}

std::size_t Mapper::run_loop_closure_scan(bool forceFullScan)
{
  if (!lc_engine_) {
    return 0;
  }
  auto lckScan = mrpt::lockHelper(lc_scan_mtx_);

  // 1) Incremental (only pairs with a new keyframe) or full scan:
  bool fullScan = forceFullScan || !params_.loop_closure_incremental;
  if (
    params_.loop_closure_full_scan_every_n > 0 &&
    lc_incremental_scans_ >= params_.loop_closure_full_scan_every_n) {
    fullScan = true;
  }

  // 2) Snapshot: current keyframe poses and the front end observations only.
  mrpt::maps::CSimpleMap snapshot;
  mola::LoopClosureAnalyzeOptions opts;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    const std::size_t n = keyframes_.size();
    if (!forceFullScan && n < lc_kf_count_at_last_scan_ + params_.loop_closure_min_new_keyframes) {
      return 0;
    }
    lc_kf_count_at_last_scan_ = n;
    for (const auto & kf : keyframes_) {
      snapshot.insert(
        mrpt::poses::CPose3DPDFGaussian::Create(kf.pose_in_map),
        std::make_shared<mrpt::obs::CSensoryFrame>(kf.observations));
    }
    // Excluded pairs (snapshot indices are keyframe indices): the closed and the
    // rejected ones.
    for (const auto & pairs : {&loop_pairs_, &rejected_loop_pairs_}) {
      for (const auto & [a, b] : *pairs) {
        opts.exclude_pairs.insert({static_cast<uint32_t>(a), static_cast<uint32_t>(b)});
      }
    }
  }
  if (snapshot.size() < 2) {
    return 0;
  }

  if (fullScan) {
    lc_incremental_scans_ = 0;
  } else {
    opts.first_new_keyframe = static_cast<uint32_t>(lc_snapshot_size_at_last_scan_);
    lc_incremental_scans_++;
  }
  opts.should_abort = [this] { return lc_should_exit_.load(); };

  mola::LoopClosureAnalyzeStats stats;
  opts.out_stats = &stats;

  // 3) Detect, off the state lock. Edges are merged in keyframe order afterwards,
  //    so the result does not depend on the detector thread scheduling.
  auto edges = lc_engine_->analyze(snapshot, opts);
  std::sort(edges.begin(), edges.end(), [](const auto & a, const auto & b) {
    return std::minmax(a.from, a.to) < std::minmax(b.from, b.to);
  });

  std::size_t merged = 0;
  std::size_t rejected = 0;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    for (const auto & e : edges) {
      if (e.from == e.to || e.from >= keyframes_.size() || e.to >= keyframes_.size()) {
        continue;
      }
      const std::pair<std::size_t, std::size_t> key = {
        std::min<std::size_t>(e.from, e.to), std::max<std::size_t>(e.from, e.to)};
      if (loop_pairs_.count(key) != 0) {
        continue;
      }
      if (params_.loop_closure_odometry_check) {
        if (const auto c = check_against_odometry_locked(key.first, key.second, e);
            c && !c->accepted) {
          if (rejected_loop_pairs_.insert(key).second) {
            MRPT_LOG_INFO_FMT(
              "Loop %zu-%zu rejected: %.2f m / %.1f deg off the odometry over %.1f m", key.first,
              key.second, c->xyz_error, c->ang_error_deg, c->distance_traveled);
            rejected++;
          }
          continue;
        }
      }
      loop_pairs_.insert(key);
      loops_.push_back({e.from, e.to, e.relative_pose});
      merged++;
    }
    if (merged > 0 && params_.loop_closure_consistency_check) {
      const std::size_t dropped = drop_inconsistent_loops_locked(loops_.size() - merged);
      merged -= std::min(merged, dropped);
    }
  }
  lc_rejected_by_odometry_ += rejected;
  lc_snapshot_size_at_last_scan_ = snapshot.size();
  lc_scans_++;
  lc_candidates_checked_ += stats.candidates_evaluated;

  if (merged > 0) {
    MRPT_LOG_INFO_STREAM(
      "Loop closure: " << merged << " new edge(s) from a " << (fullScan ? "full" : "incremental")
                       << " scan of " << snapshot.size() << " keyframes ("
                       << stats.candidates_evaluated << " candidates checked)");
    optimizer_new_data_.store(true);
    notify_optimizer();
  }
  return merged;
}

}  // namespace mola::mapper
