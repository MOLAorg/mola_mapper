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
 * @file   MapperReplay.cpp
 * @brief  Replays a recorded mapper input, with a schedule in data time.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/MapperInputLog.h>
#include <mola_mapper/MapperReplay.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <optional>
#include <type_traits>
#include <variant>

namespace mola::mapper
{
ReplayStats replay_mapper_input(
  Mapper & mapper, const std::string & inputFile, const ReplayOptions & options)
{
  const auto & params = mapper.parameters();
  ASSERTMSG_(
    !params.enable_optimizer_thread && !params.enable_loop_closure_thread,
    "replay_mapper_input(): the mapper must run without its background threads.");
  ASSERTMSG_(
    params.record_input.empty(), "replay_mapper_input(): the mapper must not record its input.");
  const double lcPeriod = options.loop_closure_period >= 0 ? options.loop_closure_period
                                                           : params.loop_closure_check_period_sec;
  const double solvePeriod =
    options.solve_period >= 0 ? options.solve_period : params.optimizer_min_period;

  ReplayStats stats;
  MapperInputReader reader(inputFile);
  std::optional<mrpt::Clock::time_point> first;
  std::optional<mrpt::Clock::time_point> lastSolve;
  std::optional<mrpt::Clock::time_point> lastScan;
  mrpt::Clock::time_point now;

  while (auto entry = reader.next()) {
    std::visit(
      [&](auto && e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, RecordedKeyframe>) {
          now = e.request.timestamp;
          if (mapper.insert_keyframe(e.request, e.gyro_bias)) {
            stats.keyframes++;
          }
        } else {
          now = e->timestamp;
          mapper.onNewObservation(e);
          stats.observations++;
        }
      },
      *entry);
    if (!std::holds_alternative<RecordedKeyframe>(*entry)) {
      continue;
    }
    // The schedule starts at the first keyframe (readings may come from other
    // clocks, or long before):
    if (!first) {
      first = now;
      lastSolve = now;
      lastScan = now;
    }

    // As the loop closure and optimizer threads, in data time: a scan (with at
    // least loop_closure_min_new_keyframes new keyframes), then a solve.
    if (
      params.loop_closure_enabled &&
      mrpt::system::timeDifference(*lastScan, now) >= std::max(0.1, lcPeriod)) {
      lastScan = now;
      mapper.run_loop_closure_scan_now(false);
      stats.loop_closure_checks++;
    }
    if (mrpt::system::timeDifference(*lastSolve, now) >= solvePeriod) {
      lastSolve = now;
      mapper.optimize_now();
      stats.solves++;
    }
    if (options.on_keyframe) {
      options.on_keyframe(stats.keyframes);
    }
  }
  if (first) {
    stats.data_duration = mrpt::system::timeDifference(*first, now);
  }
  stats.truncated = reader.truncated();
  return stats;
}

}  // namespace mola::mapper
