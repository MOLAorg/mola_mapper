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
 * @file   ImuGravityFilter.cpp
 * @brief  Filtered gravity-direction extraction from a raw high-rate IMU stream.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/ImuGravityFilter.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <cmath>

namespace mola::mapper
{
namespace
{
double norm3(const mrpt::math::TVector3D & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double angleBetweenDeg(const mrpt::math::TVector3D & a, const mrpt::math::TVector3D & b)
{
  const double na = norm3(a);
  const double nb = norm3(b);
  if (na < 1e-12 || nb < 1e-12) {
    return 0.0;
  }
  const double c = std::clamp((a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb), -1.0, 1.0);
  return mrpt::RAD2DEG(std::acos(c));
}
}  // namespace

void ImuGravityFilter::addSample(
  const mrpt::Clock::time_point & stamp, const mrpt::math::TVector3D & acc_body,
  const std::optional<mrpt::math::TVector3D> & w_body)
{
  if (!window_start_.has_value()) {
    window_start_ = stamp;
  }

  // Low-dynamics gates: keep only samples where the specific force is close to
  // pure gravity in magnitude AND the angular rate is small.
  const double aNorm = norm3(acc_body);
  const bool accStatic =
    std::abs(aNorm - params_.expected_g) <= params_.accel_tol_frac * params_.expected_g;
  bool gyroStatic = true;
  if (w_body.has_value()) {
    gyroStatic = mrpt::RAD2DEG(norm3(*w_body)) <= params_.gyro_tol_dps;
  }

  RawSample s;
  s.stamp = stamp;
  s.acc = acc_body;
  s.accepted = accStatic && gyroStatic;
  acc_.push_back(s);
}

bool ImuGravityFilter::windowReady(const mrpt::Clock::time_point & now) const
{
  if (!window_start_.has_value()) {
    return false;
  }
  return mrpt::system::timeDifference(*window_start_, now) >= params_.window_sec;
}

std::optional<ImuGravityFilter::Estimate> ImuGravityFilter::flush()
{
  const std::size_t nTotal = acc_.size();

  std::vector<mrpt::math::TVector3D> accepted;
  accepted.reserve(nTotal);
  mrpt::Clock::time_point lastStamp{};
  for (const auto & s : acc_) {
    if (s.accepted) {
      accepted.push_back(s.acc);
      lastStamp = s.stamp;
    }
  }

  // Reset the window regardless of whether we emit an estimate.
  const auto resetWindow = [&]() { clear(); };

  if (accepted.size() < params_.min_accepted) {
    resetWindow();
    return std::nullopt;
  }

  // Robust mean of the accepted accel vectors: seed = plain mean, then one
  // Huber IRLS pass to down-weight residual outliers within the window.
  const auto weightedMean =
    [](const std::vector<mrpt::math::TVector3D> & xs, const std::vector<double> & w) {
      mrpt::math::TVector3D m{0, 0, 0};
      double sw = 0;
      for (std::size_t i = 0; i < xs.size(); ++i) {
        m.x += w[i] * xs[i].x;
        m.y += w[i] * xs[i].y;
        m.z += w[i] * xs[i].z;
        sw += w[i];
      }
      if (sw > 0) {
        m.x /= sw;
        m.y /= sw;
        m.z /= sw;
      }
      return m;
    };

  std::vector<double> w(accepted.size(), 1.0);
  mrpt::math::TVector3D mean = weightedMean(accepted, w);
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    const mrpt::math::TVector3D d = {
      accepted[i].x - mean.x, accepted[i].y - mean.y, accepted[i].z - mean.z};
    const double r = norm3(d);
    w[i] = (r <= params_.huber_threshold) ? 1.0 : (params_.huber_threshold / r);
  }
  mean = weightedMean(accepted, w);

  if (norm3(mean) < 1e-6) {
    resetWindow();
    return std::nullopt;
  }

  // Earned sigma: RMS angular spread of accepted samples about the mean,
  // reduced by sqrt(n) (independent-sample averaging), clamped to [floor,ceil].
  double sumSqDeg = 0;
  for (const auto & a : accepted) {
    const double ang = angleBetweenDeg(a, mean);
    sumSqDeg += ang * ang;
  }
  const double spreadDeg = std::sqrt(sumSqDeg / static_cast<double>(accepted.size()));

  // Low-dynamics consistency gates: a trustworthy gravity reading has its
  // accepted samples tightly clustered (low spread) AND most raw samples passed
  // the per-sample gates (high acceptance fraction). Otherwise the window is
  // motion-contaminated and its mean "up" is a biased tilt that would deform the
  // map; emit nothing rather than a confident-but-wrong factor.
  const double acceptFrac =
    nTotal > 0 ? static_cast<double>(accepted.size()) / static_cast<double>(nTotal) : 0.0;
  if (
    (params_.max_spread_deg > 0 && spreadDeg > params_.max_spread_deg) ||
    (params_.min_accept_fraction > 0 && acceptFrac < params_.min_accept_fraction)) {
    thread_local const bool dbg = mrpt::get_env<bool>("MOLA_IMU_GRAVITY_FILTER_DEBUG", false);
    if (dbg) {
      printf(
        "[ImuGravityFilter] REJECT window: %zu accepted / %zu total (frac=%.2f), spread=%.2f deg\n",
        accepted.size(), nTotal, acceptFrac, spreadDeg);
    }
    resetWindow();
    return std::nullopt;
  }

  const double earned = spreadDeg / std::sqrt(static_cast<double>(accepted.size()));
  const double sigmaDeg = std::clamp(earned, params_.sigma_floor_deg, params_.sigma_ceil_deg);

  thread_local const bool MOLA_IMU_GRAVITY_FILTER_DEBUG =
    mrpt::get_env<bool>("MOLA_IMU_GRAVITY_FILTER_DEBUG", false);
  if (MOLA_IMU_GRAVITY_FILTER_DEBUG) {
    printf(
      "[ImuGravityFilter] Window: %zu accepted / %zu total, sigma_deg=%.3f (spread=%.3f) "
      "earned=%.3f\n",
      accepted.size(), nTotal, sigmaDeg, spreadDeg, earned);
  }

  Estimate e;
  const double n = norm3(mean);
  e.gravity_body_normalized = {mean.x / n, mean.y / n, mean.z / n};
  e.sigma_deg = sigmaDeg;
  e.n_accepted = accepted.size();
  e.n_total = nTotal;
  e.stamp = lastStamp;

  resetWindow();
  return e;
}

void ImuGravityFilter::clear()
{
  acc_.clear();
  window_start_.reset();
}

}  // namespace mola::mapper
