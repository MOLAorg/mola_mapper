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
 * @file   LoopOdometryCheck.cpp
 * @brief  Consistency of a proposed loop closure with the front end odometry.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/LoopOdometryCheck.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>

namespace mola::mapper
{
void LoopOdometryCheckParams::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, max_xyz);
  MCP_LOAD_OPT(cfg, max_xyz_per_m);
  MCP_LOAD_OPT(cfg, max_ang_deg);
  MCP_LOAD_OPT(cfg, max_ang_deg_per_m);
}

LoopOdometryCheck check_loop_against_odometry(
  const std::vector<mrpt::poses::CPose3D> & poseInSource, std::size_t from, std::size_t to,
  const mrpt::poses::CPose3D & loopRelPose, const LoopOdometryCheckParams & params)
{
  ASSERT_(from < poseInSource.size());
  ASSERT_(to < poseInSource.size());

  LoopOdometryCheck out;
  const std::size_t first = std::min(from, to);
  const std::size_t last = std::max(from, to);
  for (std::size_t i = first + 1; i <= last; i++) {
    out.distance_traveled +=
      (poseInSource[i].translation() - poseInSource[i - 1].translation()).norm();
  }

  // Compared in the first-to-last direction, so the result does not depend on the
  // edge order:
  const mrpt::poses::CPose3D loopFirstToLast =
    from < to ? loopRelPose : mrpt::poses::CPose3D() - loopRelPose;
  const mrpt::poses::CPose3D odoRel = poseInSource[last] - poseInSource[first];
  const mrpt::poses::CPose3D err = odoRel - loopFirstToLast;
  out.xyz_error = err.translation().norm();
  out.ang_error_deg = mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(err.getRotationMatrix()).norm());

  const double maxXyz = params.max_xyz + params.max_xyz_per_m * out.distance_traveled;
  const double maxAng = params.max_ang_deg + params.max_ang_deg_per_m * out.distance_traveled;
  out.accepted = out.xyz_error <= maxXyz && out.ang_error_deg <= maxAng;
  return out;
}

}  // namespace mola::mapper
