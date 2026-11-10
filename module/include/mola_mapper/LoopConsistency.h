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
#include <vector>

namespace mola::mapper
{
/** Two loops whose ends are near each other close a cycle with two short odometry
 *  segments: loop A, odometry from the end of A to the end of B, loop B reversed,
 *  odometry back to the start of A. Correct loops give a cycle close to the
 *  identity; a loop matched to the wrong place (repetitive scenes) does not. A
 *  loop is rejected when it is inconsistent with most of its neighbors, and with
 *  at least `min_inconsistent` of them, so a single wrong neighbor never decides. */
struct LoopConsistencyParams
{
  /// Two loops are compared when both of their ends are this close, measured
  /// along the odometry path [m].
  double neighborhood = 50.0;
  /// Cycle error limits: a floor plus a fraction of the odometry path in the cycle.
  double max_xyz = 1.0;             ///< [m]
  double max_xyz_per_m = 0.02;      ///< [m/m]
  double max_ang_deg = 3.0;         ///< [deg]
  double max_ang_deg_per_m = 0.03;  ///< [deg/m]
  uint32_t min_inconsistent = 2;

  void loadFrom(const mrpt::containers::yaml & cfg);
};

struct LoopForConsistency
{
  std::size_t from = 0;
  std::size_t to = 0;
  mrpt::poses::CPose3D relative_pose;  ///< pose of `to` seen from `from`
};

/** Front end odometry of all keyframes: poses are composable only within one chain. */
struct OdometryChains
{
  std::vector<mrpt::poses::CPose3D> pose_in_source;
  std::vector<std::size_t> chain_id;
};

/** Checks the loops [firstNew, end) one by one, in order, against the ones
 *  accepted so far (those before `firstNew`, and the new ones that passed). If any
 *  new loop was checked, the accepted ones are then re-checked, dropping the most
 *  inconsistent one while any is inconsistent with most of its neighbors (an early
 *  wrong loop, once others contradict it).
 *
 *  \return The indices of the rejected loops (new or old), in rejection order. */
[[nodiscard]] std::vector<std::size_t> find_inconsistent_loops(
  const OdometryChains & odometry, const std::vector<LoopForConsistency> & loops,
  std::size_t firstNew, const LoopConsistencyParams & params);

}  // namespace mola::mapper
