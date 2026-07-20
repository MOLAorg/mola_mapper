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
 * @file   covariance_utils.h
 * @brief  Small shared helpers to summarize SE(3) covariances.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/math/CMatrixFixed.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace mola::mapper
{

/// Returns (max position sigma [m], max orientation sigma [deg]) from an SE(3)
/// covariance using MRPT's (x, y, z, yaw, pitch, roll) convention.
inline std::pair<double, double> max_pos_and_orientation_sigma(
  const mrpt::math::CMatrixDouble66 & cov)
{
  const double maxPosSigma = std::sqrt(std::max({cov(0, 0), cov(1, 1), cov(2, 2)}));
  const double maxOriSigmaDeg =
    mrpt::RAD2DEG(std::sqrt(std::max({cov(3, 3), cov(4, 4), cov(5, 5)})));
  return {maxPosSigma, maxOriSigmaDeg};
}

}  // namespace mola::mapper
