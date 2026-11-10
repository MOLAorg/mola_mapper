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
 * @file   WheelOdometry.h
 * @brief  Wheel odometry increments between two instants.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/math/TPose2D.h>

#include <deque>

namespace mola::mapper
{
/** One wheel odometry reading: the vehicle pose in the (planar) odometry frame. */
struct WheelOdomSample
{
  mrpt::Clock::time_point stamp;
  mrpt::math::TPose2D pose;
};

enum class WheelOdomStatus
{
  Ok,
  NotYet,      //!< The readings do not reach the requested instant yet.
  Unavailable  //!< Older than the readings, or a gap longer than the limit.
};

/** Odometry pose at `t`, interpolated between the readings around it, which
 *  must be at most `maxGap` [s] apart (or the newest one, if exactly at `t`).
 *  `samples` must be time-ordered. */
WheelOdomStatus interpolate_wheel_odometry(
  const std::deque<WheelOdomSample> & samples, const mrpt::Clock::time_point & t, double maxGap,
  mrpt::math::TPose2D & out);

/** Increment of the odometry from t0 to t1 (t0 < t1), in the vehicle frame at t0. */
WheelOdomStatus wheel_odometry_increment(
  const std::deque<WheelOdomSample> & samples, const mrpt::Clock::time_point & t0,
  const mrpt::Clock::time_point & t1, double maxGap, mrpt::math::TPose2D & increment);

}  // namespace mola::mapper
