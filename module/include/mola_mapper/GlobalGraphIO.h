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
 * @file   GlobalGraphIO.h
 * @brief  Save and load the input of the global graph, to re-optimize it offline.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_mapper/GlobalGraph.h>

#include <string>
#include <utility>
#include <vector>

namespace mola::mapper
{
/** Writes the global graph input to these files:
 *  - `<prefix>.keyframes.txt`: one keyframe per line: index, stamp, source frame,
 *    pose in source (x y z yaw pitch roll) and its 6x6 covariance (36 values,
 *    MRPT order), and the initial guess in the map frame (x y z yaw pitch roll);
 *  - `<prefix>.aux.simplemap`: the attached GNSS and IMU observations, at the
 *    initial guesses;
 *  - `<prefix>.loops.txt`: from, to, relative pose and its covariance;
 *  - `<prefix>.datum.txt`: the ENU datum (latitude, longitude, height), if any;
 *  - `<prefix>.wheel_odom.txt`: the wheel odometry increments since the previous
 *    keyframe, if any: "idx dx dy dyaw".
 */
void save_global_graph_input(const GlobalGraphInput & in, const std::string & prefix);

/// Reads the files written by save_global_graph_input().
[[nodiscard]] GlobalGraphInput load_global_graph_input(const std::string & prefix);

/// Writes poses in TUM format ("t x y z qx qy qz qw").
void save_tum_trajectory(
  const std::vector<std::pair<mrpt::Clock::time_point, mrpt::poses::CPose3D>> & poses,
  const std::string & file);

/// Writes a geo-reference as YAML: datum, T_enu_to_map (x y z yaw pitch roll) and its sigmas.
void save_georeference(const mola::Georeferencing & g, const std::string & file);

}  // namespace mola::mapper
