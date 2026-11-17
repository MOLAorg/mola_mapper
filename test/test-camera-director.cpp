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
 * @file   test-camera-director.cpp
 * @brief  Unit test: the scripted GUI camera.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/CameraDirector.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>
#include <iostream>

namespace
{
using mola::mapper::CameraDirector;
using mola::mapper::CameraDirectorParams;
using mrpt::math::TBoundingBox;
using mrpt::math::TPoint3D;

CameraDirectorParams make_params()
{
  CameraDirectorParams p;
  p.overview_period = 40.0;
  p.closeup_period = 10.0;
  p.transition = 4.0;
  return p;
}

void test_schedule()
{
  const auto p = make_params();
  const auto w = [&](double t) { return CameraDirector::closeup_weight_at(p, t); };
  ASSERT_NEAR_(w(0.0), 0.0, 1e-9);
  ASSERT_NEAR_(w(39.9), 0.0, 1e-9);
  ASSERT_NEAR_(w(42.0), 0.5, 1e-9);  // half the transition
  ASSERT_NEAR_(w(44.0), 1.0, 1e-9);
  ASSERT_NEAR_(w(53.9), 1.0, 1e-9);
  ASSERT_NEAR_(w(56.0), 0.5, 1e-9);  // and back
  ASSERT_NEAR_(w(58.0 + 1.0), 0.0, 1e-9);
  // Smooth: no steps (max rate: pi / 2 / 4 s).
  for (int i = 0; i < 20000; i++) {
    const double t = 0.01 * i;
    ASSERT_LT_(std::abs(w(t + 0.01) - w(t)), 0.005);
  }
}

void test_overview_distance()
{
  auto p = make_params();
  p.overview_margin = 0;
  p.overview_min_distance = 1;
  p.fov = 90.0;  // tan(45 deg) = 1: distance = half height
  p.aspect_ratio = 2.0;
  // 100 x 20 m, seen with +y up: limited by the width.
  const TBoundingBox box({0, 0, 0}, {100, 20, 0});
  ASSERT_NEAR_(CameraDirector::overview_distance(p, box, -M_PI / 2), 25.0, 1e-6);
  // Rotated 90 deg: the 100 m side is vertical on the screen.
  ASSERT_NEAR_(CameraDirector::overview_distance(p, box, 0.0), 50.0, 1e-6);
  p.overview_min_distance = 60;
  ASSERT_NEAR_(CameraDirector::overview_distance(p, box, 0.0), 60.0, 1e-6);
}

void test_blend()
{
  auto p = make_params();
  CameraDirector d(p);

  // Nothing to look at yet:
  ASSERT_(!d.step(0.0, {}));

  CameraDirector::Inputs in;
  in.map = TBoundingBox({-100, -50, 0}, {100, 50, 0});
  in.vehicle = TPoint3D(80, 40, 0);
  in.vehicle_yaw = 0.0;

  // Overview: map center, from above, fixed azimuth.
  auto c = d.step(1.0, in);
  ASSERT_(c);
  ASSERT_NEAR_(c.value().closeup, 0.0, 1e-9);
  ASSERT_NEAR_(c.value().look_at.x, 0.0, 1e-6);
  ASSERT_NEAR_(c.value().elevation, mrpt::DEG2RAD(p.overview_elevation), 1e-9);
  ASSERT_NEAR_(c.value().azimuth, mrpt::DEG2RAD(p.overview_azimuth), 1e-9);
  ASSERT_NEAR_(
    c.value().distance,
    CameraDirector::overview_distance(p, *in.map, mrpt::DEG2RAD(p.overview_azimuth)), 1e-6);

  // Close-up: on the vehicle, behind it.
  for (int i = 10; i <= 500; i++) {
    c = d.step(0.1 * i, in);
  }
  ASSERT_(c);
  ASSERT_NEAR_(c.value().closeup, 1.0, 1e-9);
  ASSERT_NEAR_(c.value().look_at.x, 80.0, 1e-3);
  ASSERT_NEAR_(c.value().look_at.y, 40.0, 1e-3);
  ASSERT_NEAR_(std::abs(c.value().azimuth), M_PI, 1e-6);
  ASSERT_NEAR_(c.value().distance, p.closeup_distance, 1e-6);
  ASSERT_NEAR_(c.value().elevation, mrpt::DEG2RAD(p.closeup_elevation), 1e-9);

  // The end of the data: back to the overview, smoothly, and it stays there.
  in.final = true;
  auto prev = c.value();
  for (int i = 501; i <= 2000; i++) {
    c = d.step(0.1 * i, in);
    ASSERT_LT_(std::abs(c.value().closeup - prev.closeup), 0.05);
    ASSERT_LT_(std::abs(mrpt::math::wrapToPi(c.value().azimuth - prev.azimuth)), 0.1);
    prev = c.value();
  }
  ASSERT_NEAR_(c.value().closeup, 0.0, 1e-9);
  ASSERT_NEAR_(c.value().look_at.x, 0.0, 1e-6);
}

void test_heading_wraps()
{
  auto p = make_params();
  p.overview_period = 0;  // close-up only
  p.heading_time_constant = 1.0;
  CameraDirector d(p);
  CameraDirector::Inputs in;
  in.vehicle = TPoint3D(0, 0, 0);
  in.vehicle_yaw = mrpt::DEG2RAD(170.0);
  auto c = d.step(10.0, in);
  ASSERT_NEAR_(mrpt::RAD2DEG(c.value().azimuth), -10.0, 1e-6);
  // The heading crosses +-180 deg: the azimuth takes the short way (-10 -> +10).
  in.vehicle_yaw = mrpt::DEG2RAD(-170.0);
  double prev = c.value().azimuth;
  for (int i = 1; i <= 100; i++) {
    c = d.step(10.0 + 0.1 * i, in);
    ASSERT_LT_(std::abs(mrpt::math::wrapToPi(c.value().azimuth - prev)), mrpt::DEG2RAD(2.0));
    prev = c.value().azimuth;
  }
  ASSERT_NEAR_(mrpt::RAD2DEG(c.value().azimuth), 10.0, 0.01);
}

void test_opposite_azimuths()
{
  // Overview azimuth -90 deg; the vehicle heads ~-90 deg, so the close-up
  // azimuth (behind it) is ~+90 deg: their difference wobbles around +-180 deg.
  auto p = make_params();
  p.heading_time_constant = 0.01;
  CameraDirector d(p);
  CameraDirector::Inputs in;
  in.map = TBoundingBox({-100, -100, 0}, {100, 100, 0});
  in.vehicle = TPoint3D(0, 0, 0);
  double prev = 0;
  for (int i = 0; i <= 1200; i++) {
    const double t = 0.1 * i;
    in.vehicle_yaw = -M_PI / 2 + 0.05 * std::sin(0.7 * t);
    const auto c = d.step(t, in).value();
    if (i > 0) {
      ASSERT_LT_(std::abs(mrpt::math::wrapToPi(c.azimuth - prev)), 0.2);
    }
    prev = c.azimuth;
  }
}

void test_yaml()
{
  CameraDirectorParams p;
  p.loadFrom(mrpt::containers::yaml::FromText(R"(
enabled: true
closeup_distance: 25
final_overview: 30
)"));
  ASSERT_(p.enabled);
  ASSERT_NEAR_(p.closeup_distance, 25.0, 1e-9);
  ASSERT_NEAR_(p.final_overview, 30.0, 1e-9);
}

}  // namespace

int main()
{
  try {
    test_schedule();
    test_overview_distance();
    test_blend();
    test_heading_wraps();
    test_opposite_azimuths();
    test_yaml();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
