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

#include <mola_mapper/GyroIntegration.h>
#include <mrpt/poses/Lie/SO.h>

#include <algorithm>
#include <chrono>

namespace mola::mapper
{
namespace
{
/// [s], from the clock ticks: exact, unlike differences of double seconds.
double seconds_between(const mrpt::Clock::time_point & a, const mrpt::Clock::time_point & b)
{
  return std::chrono::duration<double>(b - a).count();
}
}  // namespace

std::optional<GyroIntegral> integrate_gyro(
  const std::deque<GyroSample> & samples, const mrpt::Clock::time_point & t0,
  const mrpt::Clock::time_point & t1, const mrpt::math::TVector3D & bias, double maxGap)
{
  if (samples.empty() || !(t0 < t1)) {
    return std::nullopt;
  }
  // The first reading after t0 must be close enough, and so the last one before t1:
  const auto first = std::upper_bound(
    samples.begin(), samples.end(), t0,
    [](const mrpt::Clock::time_point & t, const GyroSample & s) { return t < s.stamp; });
  if (first == samples.end()) {
    return std::nullopt;
  }
  const bool coveredStart = first != samples.begin() || seconds_between(t0, first->stamp) <= maxGap;
  if (!coveredStart || seconds_between(samples.back().stamp, t1) > maxGap) {
    return std::nullopt;
  }

  GyroIntegral out;
  out.R.setIdentity();
  mrpt::Clock::time_point prev = t0;
  for (auto it = first; it != samples.end() && prev < t1; ++it) {
    const auto segEnd = std::min(it->stamp, t1);
    const double dt = seconds_between(prev, segEnd);
    if (dt > maxGap) {
      return std::nullopt;
    }
    const mrpt::math::TVector3D w = it->w - bias;
    mrpt::math::CVectorFixedDouble<3> rv;
    rv[0] = w.x * dt;
    rv[1] = w.y * dt;
    rv[2] = w.z * dt;
    out.R = out.R * mrpt::poses::Lie::SO<3>::exp(rv);
    out.angle += rv.norm();
    prev = segEnd;
  }
  // The newest reading holds until t1:
  if (prev < t1) {
    const double dt = seconds_between(prev, t1);
    const mrpt::math::TVector3D w = samples.back().w - bias;
    mrpt::math::CVectorFixedDouble<3> rv;
    rv[0] = w.x * dt;
    rv[1] = w.y * dt;
    rv[2] = w.z * dt;
    out.R = out.R * mrpt::poses::Lie::SO<3>::exp(rv);
    out.angle += rv.norm();
  }
  out.duration = seconds_between(t0, t1);
  return out;
}

}  // namespace mola::mapper
