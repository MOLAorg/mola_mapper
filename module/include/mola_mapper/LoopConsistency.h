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
 * @file   LoopConsistency.h
 * @brief  Pairwise consistency of loop closures through the front end odometry.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace mola::mapper
{
/** Two loops whose ends are near each other close a cycle with two short odometry
 *  segments: loop A, odometry from the end of A to the end of B, loop B reversed,
 *  odometry back to the start of A. Correct loops give a cycle close to the
 *  identity; a loop matched to the wrong place (repetitive scenes) does not.
 *
 *  Loops are only compared when both of their ends are near (along the odometry
 *  path) the ends of the other one: of three visits to a place, loops pass1-pass3
 *  and pass2-pass3 are never compared with each other. */
struct LoopConsistencyParams
{
  /// Two loops are compared when both of their ends are this close, measured
  /// along the odometry path [m].
  double neighborhood = 50.0;
  /// Cycle error limits: a floor plus a fraction of the odometry path in the cycle.
  double max_xyz = 1.0;             ///< [m]
  double max_xyz_per_m = 0.04;      ///< [m/m]
  double max_ang_deg = 3.0;         ///< [deg]
  double max_ang_deg_per_m = 0.03;  ///< [deg/m]
  /// A loop is an outlier when inconsistent with at least this many of the
  /// accepted loops it is compared with, and with more than half of them (a tie
  /// is not an outlier).
  uint32_t min_inconsistent = 2;

  void loadFrom(const mrpt::containers::yaml & cfg);
};

/** A loop closure, with the front end odometry at both of its keyframes. */
struct LoopForConsistency
{
  std::size_t from = 0;                ///< keyframe index
  std::size_t to = 0;                  ///< keyframe index
  mrpt::poses::CPose3D relative_pose;  ///< pose of `to` seen from `from`
  /// Front end (odometry) poses of both keyframes, their chain, and their
  /// distance along that chain [m]. Poses are only composable within a chain.
  mrpt::poses::CPose3D from_odometry;
  mrpt::poses::CPose3D to_odometry;
  std::size_t from_chain = 0;
  std::size_t to_chain = 0;
  double from_path = 0;
  double to_path = 0;
};

/** All the loops proposed so far (accepted or not), and which ones are accepted.
 *
 *  Each group of loops connected through neighbors is decided as a whole, from
 *  scratch, every time a loop joins it: starting with all of them accepted, the
 *  outlier with the largest share of inconsistent accepted neighbors is dropped,
 *  one at a time, until there is none. So the decision depends on the set of
 *  loops, not on the order they arrived in, and a loop rejected early is accepted
 *  again once enough correct loops around it agree with it. */
class LoopConsistencyGate
{
public:
  explicit LoopConsistencyGate(const LoopConsistencyParams & params) : params_(params) {}

  /** Adds the loops and decides again the groups they join. Returns the indices
   *  (in order of addition, from 0) of the loops whose state changed: new loops
   *  that are not accepted, and older ones that were dropped or accepted again. */
  std::vector<std::size_t> add(const std::vector<LoopForConsistency> & loops);

  [[nodiscard]] std::size_t size() const { return loops_.size(); }
  [[nodiscard]] bool is_accepted(std::size_t i) const { return accepted_.at(i); }
  [[nodiscard]] const LoopForConsistency & loop(std::size_t i) const { return loops_.at(i); }
  [[nodiscard]] std::size_t rejected_count() const;

  /// Whether two loops are compared (nothing) and, if so, consistent.
  [[nodiscard]] std::optional<bool> consistent(
    const LoopForConsistency & a, const LoopForConsistency & b) const;

private:
  LoopConsistencyParams params_;
  std::vector<LoopForConsistency> loops_;  ///< from < to
  std::vector<bool> accepted_;
  /// Neighbors of each loop, and whether they are consistent with it.
  std::vector<std::vector<std::pair<std::size_t, bool>>> neighbors_;

  void decide_group(const std::vector<std::size_t> & group);
};

}  // namespace mola::mapper
