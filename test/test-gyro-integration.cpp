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
 * @file   test-gyro-integration.cpp
 * @brief  integrate_gyro(): rotation between keyframes from gyroscope readings.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GyroIntegration.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>

namespace
{
using mola::mapper::GyroSample;
using mola::mapper::integrate_gyro;

const double T0 = 1.8e9;

mrpt::Clock::time_point at(double t) { return mrpt::Clock::fromDouble(T0 + t); }

/// A 100 Hz gyro with time stamp jitter: yaw rate wz from t=1 to t=1+turnTime.
std::deque<GyroSample> turning_gyro(
  double wz, double turnTime, const mrpt::math::TVector3D & bias, double noise)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  std::uniform_real_distribution<double> jitter(-0.002, 0.002);
  std::deque<GyroSample> s;
  for (int i = 0; i <= 800; i++) {
    const double t = i * 0.01 + (i > 0 ? jitter(rng) : 0.0);
    const bool turning = t > 1.0 && t <= 1.0 + turnTime;
    s.push_back(
      {at(t),
       {bias.x + noise * n(rng), bias.y + noise * n(rng),
        (turning ? wz : 0.0) + bias.z + noise * n(rng)}});
  }
  return s;
}

void test_turn_with_bias_removed()
{
  // A 90 deg turn in 3.14 s, with a bias like that of a LiDAR internal IMU:
  const mrpt::math::TVector3D bias(0.0004, -0.0051, -0.0035);
  const auto s = turning_gyro(0.5, M_PI / 2 / 0.5, bias, 0.003);

  const auto g = integrate_gyro(s, at(0.5), at(6.0), bias, 0.1);
  ASSERT_(g.has_value());
  const auto ypr = mrpt::poses::CPose3D::FromRotationAndTranslation(
    g->R, mrpt::math::CVectorFixedDouble<3>::Zero());
  // Tolerance: the simulated turn starts and ends on 10 ms readings (0.29 deg each).
  ASSERT_NEAR_(mrpt::RAD2DEG(ypr.yaw()), 90.0, 0.4);
  ASSERT_NEAR_(mrpt::RAD2DEG(ypr.pitch()), 0.0, 0.1);
  ASSERT_NEAR_(g->duration, 5.5, 1e-9);
  ASSERT_GT_(g->angle, M_PI / 2);  // accumulated, includes the noise

  // Without removing it: 5.5 s of -0.2 deg/s on the yaw.
  const auto gb = integrate_gyro(s, at(0.5), at(6.0), {0, 0, 0}, 0.1);
  const auto yprb = mrpt::poses::CPose3D::FromRotationAndTranslation(
    gb->R, mrpt::math::CVectorFixedDouble<3>::Zero());
  ASSERT_NEAR_(mrpt::RAD2DEG(yprb.yaw()) - mrpt::RAD2DEG(ypr.yaw()), -5.5 * 0.2005, 0.05);
}

void test_exact_for_constant_rate()
{
  // A constant rate integrates exactly, whatever t0 and t1 fall between readings:
  std::deque<GyroSample> s;
  for (int i = 0; i <= 300; i++) {
    s.push_back({at(i * 0.01), {0, 0, 0.3}});
  }
  const auto g = integrate_gyro(s, at(0.123), at(2.987), {0, 0, 0}, 0.1);
  ASSERT_(g.has_value());
  const double yaw = mrpt::poses::Lie::SO<3>::log(g->R)[2];
  // (time points at this epoch are not exact in double seconds)
  ASSERT_NEAR_(g->duration, 2.987 - 0.123, 1e-4);
  ASSERT_NEAR_(yaw, 0.3 * g->duration, 1e-9);
  ASSERT_NEAR_(g->angle, 0.3 * g->duration, 1e-9);
}

/** 90 deg about x, then 90 deg about z (body rates): the result is
 *  R = Rx(90) * Rz(90), which differs from the other composition order. */
void test_two_axis_composition_order()
{
  std::deque<GyroSample> s;
  const double w = M_PI / 2;  // [rad/s], 1 s per axis
  for (int i = 0; i <= 200; i++) {
    const double t = i * 0.01;
    const bool first = t <= 1.0;
    s.push_back({at(t), first ? mrpt::math::TVector3D(w, 0, 0) : mrpt::math::TVector3D(0, 0, w)});
  }
  const auto g = integrate_gyro(s, at(0.0), at(2.0), {0, 0, 0}, 0.1);
  ASSERT_(g.has_value());

  const auto Rx = mrpt::poses::CPose3D::FromYawPitchRoll(0, 0, M_PI / 2).getRotationMatrix();
  const auto Rz = mrpt::poses::CPose3D::FromYawPitchRoll(M_PI / 2, 0, 0).getRotationMatrix();
  const mrpt::math::CMatrixDouble33 expected(Rx.asEigen() * Rz.asEigen());
  const mrpt::math::CMatrixDouble33 reversed(Rz.asEigen() * Rx.asEigen());
  const double err = (g->R.asEigen() - expected.asEigen()).norm();
  const double errReversed = (g->R.asEigen() - reversed.asEigen()).norm();
  ASSERT_LT_(err, 1e-6);
  ASSERT_GT_(errReversed, 1.0);
  ASSERT_NEAR_(g->angle, M_PI, 1e-9);
}

void test_not_covered()
{
  std::deque<GyroSample> s;
  for (int i = 0; i <= 100; i++) {
    if (i >= 40 && i < 60) {
      continue;  // a 0.2 s gap
    }
    s.push_back({at(i * 0.01), {0, 0, 0.1}});
  }
  ASSERT_(integrate_gyro(s, at(0.0), at(0.3), {0, 0, 0}, 0.1).has_value());
  ASSERT_(!integrate_gyro(s, at(0.1), at(0.9), {0, 0, 0}, 0.1).has_value());   // spans the gap
  ASSERT_(!integrate_gyro(s, at(-1.0), at(0.3), {0, 0, 0}, 0.1).has_value());  // before data
  ASSERT_(!integrate_gyro(s, at(0.7), at(2.0), {0, 0, 0}, 0.1).has_value());   // after data
  ASSERT_(!integrate_gyro(s, at(0.3), at(0.3), {0, 0, 0}, 0.1).has_value());   // empty
  ASSERT_(!integrate_gyro({}, at(0.0), at(0.3), {0, 0, 0}, 0.1).has_value());
}
}  // namespace

int main()
{
  try {
    test_turn_with_bias_removed();
    test_exact_for_constant_rate();
    test_two_axis_composition_order();
    test_not_covered();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
