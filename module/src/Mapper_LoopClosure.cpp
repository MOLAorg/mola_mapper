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

LoopForConsistency Mapper::loop_for_consistency_locked(const GlobalGraphLoopEdge & e)
{
  // Distance along each chain, for the keyframes not done yet:
  for (std::size_t i = keyframe_path_.size(); i < keyframes_.size(); i++) {
    const auto & kf = keyframes_[i];
    double path = 0;
    if (const auto it = chain_last_keyframe_.find(kf.source_frame);
        it != chain_last_keyframe_.end()) {
      const auto prev = it->second;
      path = keyframe_path_[prev] + (kf.pose_in_source.mean.translation() -
                                     keyframes_[prev].pose_in_source.mean.translation())
                                      .norm();
    }
    keyframe_path_.push_back(path);
    chain_last_keyframe_[kf.source_frame] = i;
  }
  const auto chainOf = [this](std::size_t i) {
    return chain_ids_.try_emplace(keyframes_[i].source_frame, chain_ids_.size()).first->second;
  };
  LoopForConsistency c;
  c.from = e.from;
  c.to = e.to;
  c.relative_pose = e.relative_pose.mean;
  c.from_odometry = keyframes_[e.from].pose_in_source.mean;
  c.to_odometry = keyframes_[e.to].pose_in_source.mean;
  c.from_chain = chainOf(e.from);
  c.to_chain = chainOf(e.to);
  c.from_path = keyframe_path_[e.from];
  c.to_path = keyframe_path_[e.to];
  return c;
}

std::size_t Mapper::add_loop_closures(const std::vector<mola::ProposedLoopEdge> & edges)
{
  auto lckScan = mrpt::lockHelper(lc_scan_mtx_);
  return merge_loop_edges(edges);
}

std::size_t Mapper::merge_loop_edges(const std::vector<mola::ProposedLoopEdge> & edges)
{
  const bool useGate = params_.loop_closure_consistency_check;
  std::size_t merged = 0;
  std::size_t rejected = 0;
  std::vector<GlobalGraphLoopEdge> newEdges;
  std::vector<LoopForConsistency> newLoops;
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
      // Kept for good: a loop rejected by the consistency check may be accepted
      // later, so its pair is not proposed again either.
      loop_pairs_.insert(key);
      const GlobalGraphLoopEdge edge = {e.from, e.to, e.relative_pose};
      if (!useGate) {
        loops_.push_back(edge);
        loops_revision_++;
        loops_added_++;
        merged++;
        continue;
      }
      newEdges.push_back(edge);
      newLoops.push_back(loop_for_consistency_locked(edge));
    }
  }
  lc_rejected_by_odometry_ += rejected;

  bool changed = merged > 0;
  if (!newEdges.empty()) {
    // Off the state lock (this thread is the only writer of loops):
    if (!lc_gate_) {
      lc_gate_.emplace(params_.loop_closure_consistency);
    }
    const std::size_t firstNew = lc_gate_->size();
    const auto changes = lc_gate_->add(newLoops);
    lc_gate_edges_.insert(lc_gate_edges_.end(), newEdges.begin(), newEdges.end());

    std::size_t added = newEdges.size();
    std::size_t removed = 0;
    for (const auto k : changes) {
      const auto & l = lc_gate_edges_.at(k);
      const char * what = nullptr;
      if (k >= firstNew) {
        added--;
        what = "rejected: inconsistent with most of its neighbor loops";
      } else if (lc_gate_->is_accepted(k)) {
        added++;
        what = "accepted again: consistent with most of its neighbor loops";
      } else {
        removed++;
        what = "dropped: inconsistent with most of its neighbor loops";
      }
      MRPT_LOG_INFO_FMT("Loop %zu-%zu %s", std::min(l.from, l.to), std::max(l.from, l.to), what);
    }
    std::vector<GlobalGraphLoopEdge> accepted;
    std::vector<std::pair<std::size_t, std::size_t>> rejectedPairs;
    for (std::size_t k = 0; k < lc_gate_->size(); k++) {
      const auto & l = lc_gate_edges_[k];
      if (lc_gate_->is_accepted(k)) {
        accepted.push_back(l);
      } else {
        rejectedPairs.emplace_back(std::min(l.from, l.to), std::max(l.from, l.to));
      }
    }
    {
      auto lck = mrpt::lockHelper(state_mtx_);
      loops_ = std::move(accepted);
      consistency_rejected_pairs_ = std::move(rejectedPairs);
      if (added > 0 || removed > 0) {
        loops_revision_++;
        loops_added_ += added;
      }
    }
    lc_currently_rejected_by_consistency_ = lc_gate_->rejected_count();
    merged = added;
    changed = added > 0 || removed > 0;
  }
  if (changed) {
    optimizer_new_data_.store(true);
    notify_optimizer();
  }
  return merged;
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

  const std::size_t merged = merge_loop_edges(edges);
  lc_snapshot_size_at_last_scan_ = snapshot.size();
  lc_scans_++;
  lc_candidates_checked_ += stats.candidates_evaluated;

  if (merged > 0) {
    MRPT_LOG_INFO_STREAM(
      "Loop closure: " << merged << " new edge(s) from a " << (fullScan ? "full" : "incremental")
                       << " scan of " << snapshot.size() << " keyframes ("
                       << stats.candidates_evaluated << " candidates checked)");
  }
  return merged;
}

}  // namespace mola::mapper
