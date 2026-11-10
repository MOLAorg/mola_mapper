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
 * @file   WheelOdometry.cpp
 * @brief  Wheel odometry increments between two instants.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/WheelOdometry.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>

#include <algorithm>
#include <chrono>

namespace mola::mapper
{
namespace
{
double seconds_between(const mrpt::Clock::time_point & a, const mrpt::Clock::time_point & b)
{
  return std::chrono::duration<double>(b - a).count();
}
}  // namespace

WheelOdomStatus interpolate_wheel_odometry(
  const std::deque<WheelOdomSample> & samples, const mrpt::Clock::time_point & t, double maxGap,
  mrpt::math::TPose2D & out)
{
  if (samples.empty() || samples.back().stamp < t) {
    return WheelOdomStatus::NotYet;
  }
  // First reading at or after t:
  const auto after = std::lower_bound(
    samples.begin(), samples.end(), t,
    [](const WheelOdomSample & s, const mrpt::Clock::time_point & tt) { return s.stamp < tt; });
  if (after->stamp == t) {
    out = after->pose;
    return WheelOdomStatus::Ok;
  }
  if (after == samples.begin()) {
    return WheelOdomStatus::Unavailable;
  }
  const auto before = std::prev(after);
  const double dt = seconds_between(before->stamp, after->stamp);
  if (dt > maxGap) {
    return WheelOdomStatus::Unavailable;
  }
  const double a = seconds_between(before->stamp, t) / dt;
  const auto & p0 = before->pose;
  const auto & p1 = after->pose;
  out.x = p0.x + a * (p1.x - p0.x);
  out.y = p0.y + a * (p1.y - p0.y);
  out.phi = mrpt::math::wrapToPi(p0.phi + a * mrpt::math::wrapToPi(p1.phi - p0.phi));
  return WheelOdomStatus::Ok;
}

WheelOdomStatus wheel_odometry_increment(
  const std::deque<WheelOdomSample> & samples, const mrpt::Clock::time_point & t0,
  const mrpt::Clock::time_point & t1, double maxGap, mrpt::math::TPose2D & increment)
{
  if (!(t0 < t1)) {
    return WheelOdomStatus::Unavailable;
  }
  mrpt::math::TPose2D p1;
  const auto s1 = interpolate_wheel_odometry(samples, t1, maxGap, p1);
  if (s1 != WheelOdomStatus::Ok) {
    return s1;
  }
  mrpt::math::TPose2D p0;
  if (interpolate_wheel_odometry(samples, t0, maxGap, p0) != WheelOdomStatus::Ok) {
    return WheelOdomStatus::Unavailable;
  }
  // Gaps inside the interval (e.g. the odometry stopped) invalidate it too:
  const auto first = std::lower_bound(
    samples.begin(), samples.end(), t0,
    [](const WheelOdomSample & s, const mrpt::Clock::time_point & tt) { return s.stamp < tt; });
  for (auto it = first; it != samples.end() && it->stamp < t1; ++it) {
    if (
      std::next(it) != samples.end() && seconds_between(it->stamp, std::next(it)->stamp) > maxGap) {
      return WheelOdomStatus::Unavailable;
    }
  }
  increment = (mrpt::poses::CPose2D(p1) - mrpt::poses::CPose2D(p0)).asTPose();
  return WheelOdomStatus::Ok;
}

}  // namespace mola::mapper
