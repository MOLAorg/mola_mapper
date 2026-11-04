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
 * @file   LoopOdometryCheck.h
 * @brief  Consistency of a proposed loop closure with the front end odometry.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <vector>

namespace mola::mapper
{
/** Limits of the disagreement between a loop closure and the odometry chain: a
 *  floor plus a fraction of the distance traveled between the two keyframes. */
struct LoopOdometryCheckParams
{
  double max_xyz = 2.0;             ///< [m]
  double max_xyz_per_m = 0.10;      ///< [m/m]
  double max_ang_deg = 5.0;         ///< [deg]
  double max_ang_deg_per_m = 0.03;  ///< [deg/m]

  void loadFrom(const mrpt::containers::yaml & cfg);
};

struct LoopOdometryCheck
{
  double distance_traveled = 0;  ///< along the odometry chain [m]
  double xyz_error = 0;          ///< [m]
  double ang_error_deg = 0;      ///< [deg]
  bool accepted = true;
};

/** Compares a loop closure (`loopRelPose`: pose of keyframe `to` seen from
 *  keyframe `from`) with the same relative pose from the odometry chain
 *  `poseInSource` (one pose per keyframe, all in the same odometry frame). Loops
 *  between keyframes not far apart along the chain must agree with it, since the
 *  odometry drift over a short distance is small; a loop that does not is a false
 *  match (e.g. repetitive geometry). */
[[nodiscard]] LoopOdometryCheck check_loop_against_odometry(
  const std::vector<mrpt::poses::CPose3D> & poseInSource, std::size_t from, std::size_t to,
  const mrpt::poses::CPose3D & loopRelPose, const LoopOdometryCheckParams & params);

}  // namespace mola::mapper
