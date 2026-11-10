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
 * @file   LoopConsistency.cpp
 * @brief  Pairwise consistency of loop closures through the front end odometry.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/LoopConsistency.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace mola::mapper
{
void LoopConsistencyParams::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, neighborhood);
  MCP_LOAD_OPT(cfg, max_xyz);
  MCP_LOAD_OPT(cfg, max_xyz_per_m);
  MCP_LOAD_OPT(cfg, max_ang_deg);
  MCP_LOAD_OPT(cfg, max_ang_deg_per_m);
  MCP_LOAD_OPT(cfg, min_inconsistent);
}

namespace
{
using mrpt::poses::CPose3D;

/// Each loop from its older keyframe, so the ends of two loops match by age.
LoopForConsistency oldest_first(LoopForConsistency l)
{
  if (l.from > l.to) {
    std::swap(l.from, l.to);
    std::swap(l.from_odometry, l.to_odometry);
    std::swap(l.from_chain, l.to_chain);
    std::swap(l.from_path, l.to_path);
    l.relative_pose = CPose3D() - l.relative_pose;
  }
  return l;
}
}  // namespace

std::optional<bool> LoopConsistencyGate::consistent(
  const LoopForConsistency & a, const LoopForConsistency & b) const
{
  // The cycle error is measured in the frame of x.from: the same x for either
  // order of the arguments, so the result is symmetric.
  const bool aFirst = std::make_pair(a.from, a.to) <= std::make_pair(b.from, b.to);
  const auto & x = aFirst ? a : b;
  const auto & y = aFirst ? b : a;
  if (x.from_chain != y.from_chain || x.to_chain != y.to_chain) {
    return std::nullopt;
  }
  const double dFrom = std::abs(x.from_path - y.from_path);
  const double dTo = std::abs(x.to_path - y.to_path);
  if (dFrom > params_.neighborhood || dTo > params_.neighborhood) {
    return std::nullopt;
  }
  // x.from -> x.to -> y.to -> y.from -> x.from:
  const CPose3D cycle = x.relative_pose + (y.to_odometry - x.to_odometry) +
                        (CPose3D() - y.relative_pose) + (x.from_odometry - y.from_odometry);
  const double pathInCycle = dFrom + dTo;
  const double errXyz = cycle.translation().norm();
  const double errAngDeg =
    mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(cycle.getRotationMatrix()).norm());
  return errXyz <= params_.max_xyz + params_.max_xyz_per_m * pathInCycle &&
         errAngDeg <= params_.max_ang_deg + params_.max_ang_deg_per_m * pathInCycle;
}

std::size_t LoopConsistencyGate::rejected_count() const
{
  return static_cast<std::size_t>(std::count(accepted_.begin(), accepted_.end(), false));
}

std::vector<std::size_t> LoopConsistencyGate::add(const std::vector<LoopForConsistency> & loops)
{
  const std::size_t firstNew = loops_.size();
  for (const auto & l : loops) {
    const auto n = oldest_first(l);
    const std::size_t k = loops_.size();
    neighbors_.emplace_back();
    // Distances along the path first: most loops are not neighbors.
    for (std::size_t j = 0; j < k; j++) {
      if (const auto c = consistent(n, loops_[j]); c.has_value()) {
        neighbors_[k].emplace_back(j, *c);
        neighbors_[j].emplace_back(k, *c);
      }
    }
    loops_.push_back(n);
    accepted_.push_back(true);
  }

  // The groups the new loops joined, all their members, decided from scratch:
  const std::vector<bool> before = accepted_;
  std::vector<bool> visited(loops_.size(), false);
  for (std::size_t k = firstNew; k < loops_.size(); k++) {
    if (visited[k]) {
      continue;
    }
    std::vector<std::size_t> group = {k};
    visited[k] = true;
    for (std::size_t q = 0; q < group.size(); q++) {
      for (const auto & [j, c] : neighbors_[group[q]]) {
        if (!visited[j]) {
          visited[j] = true;
          group.push_back(j);
        }
      }
    }
    decide_group(group);
  }

  std::vector<std::size_t> changed;
  for (std::size_t k = 0; k < loops_.size(); k++) {
    const bool wasAccepted = k < firstNew ? before[k] : true;
    if (accepted_[k] != wasAccepted) {
      changed.push_back(k);
    }
  }
  return changed;
}

void LoopConsistencyGate::decide_group(const std::vector<std::size_t> & group)
{
  // Votes among the accepted members, all of them accepted to begin with:
  std::vector<std::size_t> numNeighbors(loops_.size(), 0);
  std::vector<std::size_t> numInconsistent(loops_.size(), 0);
  for (const auto k : group) {
    accepted_[k] = true;
  }
  for (const auto k : group) {
    numNeighbors[k] = neighbors_[k].size();
    for (const auto & [j, c] : neighbors_[k]) {
      if (!c) {
        numInconsistent[k]++;
      }
    }
  }
  const auto isOutlier = [&](std::size_t k) {
    return numInconsistent[k] >= params_.min_inconsistent &&
           2 * numInconsistent[k] > numNeighbors[k];
  };

  // Whether k is a worse outlier than w: a larger share of inconsistent
  // neighbors, then more of them, then the oldest keyframe pair (an arbitrary
  // but fixed rule, independent of the order the loops arrived in).
  const auto worse = [&](std::size_t k, std::size_t w) {
    const auto a = numInconsistent[k] * numNeighbors[w];
    const auto b = numInconsistent[w] * numNeighbors[k];
    if (a != b) {
      return a > b;
    }
    if (numInconsistent[k] != numInconsistent[w]) {
      return numInconsistent[k] > numInconsistent[w];
    }
    return std::make_pair(loops_[k].from, loops_[k].to) <
           std::make_pair(loops_[w].from, loops_[w].to);
  };

  for (;;) {
    std::optional<std::size_t> worst;
    for (const auto k : group) {
      if (accepted_[k] && isOutlier(k) && (!worst || worse(k, *worst))) {
        worst = k;
      }
    }
    if (!worst) {
      break;
    }
    accepted_[*worst] = false;
    for (const auto & [j, c] : neighbors_[*worst]) {
      numNeighbors[j]--;
      if (!c) {
        numInconsistent[j]--;
      }
    }
  }

  // A loop dropped early may have no inconsistent accepted neighbor left once
  // the ones that outvoted it are dropped too: accepted again, oldest keyframe
  // pair first (each one only adds consistent votes to the accepted ones).
  std::vector<std::size_t> rejected;
  for (const auto k : group) {
    if (!accepted_[k]) {
      rejected.push_back(k);
    }
  }
  std::sort(rejected.begin(), rejected.end(), [this](std::size_t a, std::size_t b) {
    return std::make_pair(loops_[a].from, loops_[a].to) <
           std::make_pair(loops_[b].from, loops_[b].to);
  });
  for (const auto k : rejected) {
    const bool contradicted = std::any_of(
      neighbors_[k].begin(), neighbors_[k].end(),
      [this](const auto & n) { return accepted_[n.first] && !n.second; });
    if (!contradicted) {
      accepted_[k] = true;
    }
  }
}

}  // namespace mola::mapper
