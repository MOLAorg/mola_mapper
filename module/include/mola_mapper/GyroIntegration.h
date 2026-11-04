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
 * @file   GyroIntegration.h
 * @brief  Integration of gyroscope readings between two instants.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

#include <deque>
#include <optional>

namespace mola::mapper
{
/** One gyroscope reading (sensor frame) [rad/s]. */
struct GyroSample
{
  mrpt::Clock::time_point stamp;
  mrpt::math::TVector3D w;
};

/** The rotation of a gyroscope between two instants. */
struct GyroIntegral
{
  mrpt::math::CMatrixDouble33 R;  //!< from t0 to t1, sensor frame
  double angle = 0;               //!< accumulated rotation angle, sum of |w| dt [rad]
  double duration = 0;            //!< [s]
};

/** Integrates time-ordered gyroscope readings between t0 < t1, after removing
 *  `bias`. Each reading holds over the interval since the previous one. Returns
 *  nothing if the readings do not cover [t0, t1] without gaps longer than
 *  `maxGap` [s].
 */
std::optional<GyroIntegral> integrate_gyro(
  const std::deque<GyroSample> & samples, const mrpt::Clock::time_point & t0,
  const mrpt::Clock::time_point & t1, const mrpt::math::TVector3D & bias, double maxGap);

}  // namespace mola::mapper
