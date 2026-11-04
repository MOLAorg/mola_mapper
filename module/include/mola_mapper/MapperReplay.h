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
 * @file   MapperReplay.h
 * @brief  Replays a recorded mapper input, with a schedule in data time, so the
 *         result does not depend on the computer or its load.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_mapper/Mapper.h>

#include <cstddef>
#include <functional>
#include <string>

namespace mola::mapper
{
struct ReplayOptions
{
  /// Data time between loop closure scans [s] (<0: loop_closure_check_period_sec).
  double loop_closure_period = -1;
  /// Data time between solves [s] (<0: optimizer_min_period).
  double solve_period = -1;
  /// Called after each keyframe, with the number of keyframes so far.
  std::function<void(std::size_t)> on_keyframe;
};

struct ReplayStats
{
  std::size_t observations = 0;
  std::size_t keyframes = 0;
  std::size_t loop_closure_scans = 0;
  std::size_t solves = 0;
  double data_duration = 0;  ///< [s] from the first entry to the last one
};

/** Feeds a mapper with the entries of a recorded input (Parameters::record_input)
 *  in their order. The mapper must have been initialized without its background
 *  threads (enable_optimizer_thread and enable_loop_closure_thread false): the
 *  loop closure scans and solves run here instead, as the threads would, but at
 *  fixed periods of data time. finalize() is not called. */
ReplayStats replay_mapper_input(
  Mapper & mapper, const std::string & inputFile, const ReplayOptions & options = {});

}  // namespace mola::mapper
