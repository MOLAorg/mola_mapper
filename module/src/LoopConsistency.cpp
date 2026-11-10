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
#include <map>
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

class Checker
{
public:
  Checker(
    const OdometryChains & odo, const std::vector<LoopForConsistency> & loops,
    const LoopConsistencyParams & p)
  : odo_(odo), p_(p)
  {
    // Path length along each chain:
    const std::size_t n = odo.pose_in_source.size();
    ASSERT_EQUAL_(odo.chain_id.size(), n);
    path_.assign(n, 0.0);
    std::map<std::size_t, std::size_t> lastOfChain;
    for (std::size_t i = 0; i < n; i++) {
      const auto it = lastOfChain.find(odo.chain_id[i]);
      if (it != lastOfChain.end()) {
        const auto prev = it->second;
        path_[i] =
          path_[prev] +
          (odo.pose_in_source[i].translation() - odo.pose_in_source[prev].translation()).norm();
      }
      lastOfChain[odo.chain_id[i]] = i;
    }
    // Each loop from its older keyframe, so the ends of two loops match by age:
    loops_.reserve(loops.size());
    for (auto l : loops) {
      ASSERT_LT_(l.from, n);
      ASSERT_LT_(l.to, n);
      if (l.from > l.to) {
        std::swap(l.from, l.to);
        l.relative_pose = CPose3D() - l.relative_pose;
      }
      loops_.push_back(l);
    }
  }

  /// Nothing if the loops are not neighbors (or cannot be composed).
  std::optional<bool> consistent(std::size_t a, std::size_t b)
  {
    const auto key = std::minmax(a, b);
    if (const auto it = cache_.find(key); it != cache_.end()) {
      return it->second;
    }
    const auto r = compute(loops_[a], loops_[b]);
    cache_[key] = r;
    return r;
  }

private:
  const OdometryChains & odo_;
  const LoopConsistencyParams & p_;
  std::vector<LoopForConsistency> loops_;
  std::vector<double> path_;
  std::map<std::pair<std::size_t, std::size_t>, std::optional<bool>> cache_;

  bool sameChain(std::size_t i, std::size_t j) const
  {
    return odo_.chain_id[i] == odo_.chain_id[j];
  }

  CPose3D odometry(std::size_t i, std::size_t j) const
  {
    return odo_.pose_in_source[j] - odo_.pose_in_source[i];
  }

  std::optional<bool> compute(const LoopForConsistency & x, LoopForConsistency y) const
  {
    // Match the ends of both loops by chain, reversing `y` if needed:
    if (!sameChain(x.from, y.from) || !sameChain(x.to, y.to)) {
      std::swap(y.from, y.to);
      y.relative_pose = CPose3D() - y.relative_pose;
      if (!sameChain(x.from, y.from) || !sameChain(x.to, y.to)) {
        return std::nullopt;
      }
    }
    const double dFrom = std::abs(path_[x.from] - path_[y.from]);
    const double dTo = std::abs(path_[x.to] - path_[y.to]);
    if (dFrom > p_.neighborhood || dTo > p_.neighborhood) {
      return std::nullopt;
    }
    const CPose3D cycle = x.relative_pose + odometry(x.to, y.to) + (CPose3D() - y.relative_pose) +
                          odometry(y.from, x.from);
    const double pathInCycle = dFrom + dTo;
    const double errXyz = cycle.translation().norm();
    const double errAngDeg =
      mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(cycle.getRotationMatrix()).norm());
    return errXyz <= p_.max_xyz + p_.max_xyz_per_m * pathInCycle &&
           errAngDeg <= p_.max_ang_deg + p_.max_ang_deg_per_m * pathInCycle;
  }
};

struct Votes
{
  std::size_t neighbors = 0;
  std::size_t inconsistent = 0;
};

Votes count_votes(Checker & c, std::size_t k, const std::vector<std::size_t> & others)
{
  Votes v;
  for (const auto o : others) {
    if (o == k) {
      continue;
    }
    if (const auto r = c.consistent(k, o); r.has_value()) {
      v.neighbors++;
      if (!*r) {
        v.inconsistent++;
      }
    }
  }
  return v;
}

bool is_outlier(const Votes & v, const LoopConsistencyParams & p)
{
  return v.inconsistent >= p.min_inconsistent && 2 * v.inconsistent > v.neighbors;
}

}  // namespace

std::vector<std::size_t> find_inconsistent_loops(
  const OdometryChains & odometry, const std::vector<LoopForConsistency> & loops,
  std::size_t firstNew, const LoopConsistencyParams & params)
{
  std::vector<std::size_t> rejected;
  if (firstNew >= loops.size()) {
    return rejected;
  }
  Checker checker(odometry, loops, params);

  std::vector<std::size_t> accepted;
  for (std::size_t k = 0; k < firstNew; k++) {
    accepted.push_back(k);
  }
  for (std::size_t k = firstNew; k < loops.size(); k++) {
    if (is_outlier(count_votes(checker, k, accepted), params)) {
      rejected.push_back(k);
    } else {
      accepted.push_back(k);
    }
  }

  // Re-check: drop the worst one (largest share of inconsistent neighbors) at a time.
  for (;;) {
    std::optional<std::size_t> worst;
    double worstShare = 0;
    for (const auto k : accepted) {
      const auto v = count_votes(checker, k, accepted);
      if (!is_outlier(v, params)) {
        continue;
      }
      const double share = static_cast<double>(v.inconsistent) / static_cast<double>(v.neighbors);
      if (!worst || share > worstShare) {
        worst = k;
        worstShare = share;
      }
    }
    if (!worst) {
      break;
    }
    accepted.erase(std::find(accepted.begin(), accepted.end(), *worst));
    rejected.push_back(*worst);
  }
  return rejected;
}

}  // namespace mola::mapper
