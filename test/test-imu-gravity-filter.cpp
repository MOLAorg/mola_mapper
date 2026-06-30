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
 * @file   test-imu-gravity-filter.cpp
 * @brief  Unit tests for ImuGravityFilter (filtered low-dynamics gravity).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/ImuGravityFilter.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>

using mola::mapper::ImuGravityFilter;

namespace
{
constexpr double g = 9.81;

ImuGravityFilter makeFilter()
{
  ImuGravityFilter::Parameters p;
  p.window_sec = 1.0;
  p.accel_tol_frac = 0.05;
  p.gyro_tol_dps = 3.0;
  p.min_accepted = 10;
  p.sigma_floor_deg = 0.2;
  p.sigma_ceil_deg = 5.0;
  return ImuGravityFilter(p);
}

double angleDeg(const mrpt::math::TVector3D & a, const mrpt::math::TVector3D & b)
{
  const double c =
    (a.x * b.x + a.y * b.y + a.z * b.z) /
    (std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) * std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z));
  return mrpt::RAD2DEG(std::acos(std::clamp(c, -1.0, 1.0)));
}

// (1) Clean near-static stream: gravity recovered with a tight earned sigma.
void test_clean_static()
{
  auto f = makeFilter();
  auto & rng = mrpt::random::getRandomGenerator();
  for (int i = 0; i < 100; i++) {
    const auto t = mrpt::Clock::fromDouble(0.01 * (i + 1));
    const mrpt::math::TVector3D a = {
      rng.drawGaussian1D(0, 0.05), rng.drawGaussian1D(0, 0.05), g + rng.drawGaussian1D(0, 0.05)};
    const mrpt::math::TVector3D w = {0, 0, 0};
    f.addSample(t, a, w);
  }
  const auto est = f.flush();
  ASSERT_(est.has_value());
  std::cout << "[clean] n=" << est->n_accepted << "/" << est->n_total << " sigma=" << est->sigma_deg
            << " deg, dir-err=" << angleDeg(est->gravity_body_normalized, {0, 0, 1}) << " deg\n";
  ASSERT_EQUAL_(est->n_accepted, est->n_total);
  ASSERT_LT_(angleDeg(est->gravity_body_normalized, {0, 0, 1}), 1.0);
  // Many agreeing samples -> sigma reduced well below 1 sample's ~0.3 deg spread.
  ASSERT_LT_(est->sigma_deg, 0.5);
}

// (2) Mixed stream: dynamic samples (high |a| or high gyro) are rejected, the
//     clean ones still recover gravity.
void test_rejects_dynamic()
{
  auto f = makeFilter();
  std::size_t expectedAccepted = 0;
  for (int i = 0; i < 100; i++) {
    const auto t = mrpt::Clock::fromDouble(0.01 * (i + 1));
    if (i % 2 == 0) {
      // clean
      f.addSample(t, {0, 0, g}, mrpt::math::TVector3D{0, 0, 0});
      expectedAccepted++;
    } else if (i % 4 == 1) {
      // high linear acceleration (|a| far from g): horizontal 5 m/s^2 added
      f.addSample(t, {5.0, 0, g}, mrpt::math::TVector3D{0, 0, 0});
    } else {
      // high angular rate (10 deg/s > 3 deg/s gate)
      f.addSample(t, {0, 0, g}, mrpt::math::TVector3D{0, 0, mrpt::DEG2RAD(10.0)});
    }
  }
  const auto est = f.flush();
  ASSERT_(est.has_value());
  std::cout << "[mixed] n=" << est->n_accepted << "/" << est->n_total
            << " dir-err=" << angleDeg(est->gravity_body_normalized, {0, 0, 1}) << " deg\n";
  ASSERT_EQUAL_(est->n_accepted, expectedAccepted);
  ASSERT_LT_(angleDeg(est->gravity_body_normalized, {0, 0, 1}), 0.5);
}

// (3) All-dynamic stream: nothing passes the gates -> no estimate emitted.
void test_all_dynamic_returns_nullopt()
{
  auto f = makeFilter();
  for (int i = 0; i < 100; i++) {
    const auto t = mrpt::Clock::fromDouble(0.01 * (i + 1));
    f.addSample(t, {6.0, 0, g}, mrpt::math::TVector3D{0, 0, mrpt::DEG2RAD(20.0)});
  }
  const auto est = f.flush();
  ASSERT_(!est.has_value());
}

// (4) Tilted gravity direction is recovered (sensor mounted/parked on a slope).
void test_recovers_tilted_direction()
{
  auto f = makeFilter();
  const double tilt = mrpt::DEG2RAD(20.0);
  const mrpt::math::TVector3D gravDir = {std::sin(tilt), 0, std::cos(tilt)};
  for (int i = 0; i < 100; i++) {
    const auto t = mrpt::Clock::fromDouble(0.01 * (i + 1));
    f.addSample(t, {g * gravDir.x, g * gravDir.y, g * gravDir.z}, mrpt::math::TVector3D{0, 0, 0});
  }
  const auto est = f.flush();
  ASSERT_(est.has_value());
  ASSERT_LT_(angleDeg(est->gravity_body_normalized, gravDir), 0.5);
}

// (5) windowReady() fires only after window_sec of data.
void test_window_ready()
{
  auto f = makeFilter();
  f.addSample(mrpt::Clock::fromDouble(10.0), {0, 0, g}, std::nullopt);
  ASSERT_(!f.windowReady(mrpt::Clock::fromDouble(10.5)));
  ASSERT_(f.windowReady(mrpt::Clock::fromDouble(11.01)));
  f.clear();
  ASSERT_EQUAL_(f.bufferedSamples(), 0U);
}
}  // namespace

int main()
{
  try {
    test_clean_static();
    test_rejects_dynamic();
    test_all_dynamic_returns_nullopt();
    test_recovers_tilted_direction();
    test_window_ready();
    std::cout << "All ImuGravityFilter tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
