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
 * @file   test-loop-odometry-check.cpp
 * @brief  Unit test: loop closures that contradict the odometry chain are rejected.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/LoopOdometryCheck.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
using mola::mapper::check_loop_against_odometry;
using mrpt::poses::CPose3D;

/** A 100 x 50 m rectangle driven twice, a keyframe per meter. Ground truth, and
 *  odometry with 0.02 deg of heading drift and 1 % of scale error per meter. */
struct Run
{
  std::vector<CPose3D> gt;
  std::vector<CPose3D> odo;
};

Run make_run()
{
  Run r;
  CPose3D gt;
  CPose3D odo;
  for (int lap = 0; lap < 2; lap++) {
    for (const int side : {100, 50, 100, 50}) {
      for (int i = 0; i < side; i++) {
        r.gt.push_back(gt);
        r.odo.push_back(odo);
        const bool corner = (i == side - 1);
        const CPose3D step(1.0, 0, 0, corner ? M_PI / 2 : 0.0, 0, 0);
        gt = gt + step;
        odo = odo + CPose3D(1.01, 0, 0, step.yaw() + mrpt::DEG2RAD(0.02), 0, 0);
      }
    }
  }
  return r;
}

void test_true_loops_accepted()
{
  const auto r = make_run();
  const mola::mapper::LoopOdometryCheckParams p;
  // The same place one lap later, 5 m further along (across a corner for the
  // second one), so the loop relative pose is not the identity:
  for (const std::size_t i : {10U, 97U, 200U}) {
    const std::size_t j = i + 305;
    const auto c = check_loop_against_odometry(r.odo, i, j, r.gt[j] - r.gt[i], p);
    std::cout << "true loop " << i << "-" << j << ": " << c.xyz_error << " m, " << c.ang_error_deg
              << " deg off over " << c.distance_traveled << " m\n";
    ASSERT_(c.accepted);
    ASSERT_NEAR_(c.distance_traveled, 305 * 1.01, 1e-6);
    ASSERT_GT_(c.xyz_error, 2.0);  // the odometry drift is real, and tolerated

    // The same edge stated the other way around gives the same verdict and errors:
    const auto cr = check_loop_against_odometry(r.odo, j, i, r.gt[i] - r.gt[j], p);
    ASSERT_(cr.accepted);
    ASSERT_NEAR_(cr.xyz_error, c.xyz_error, 1e-9);
    ASSERT_NEAR_(cr.ang_error_deg, c.ang_error_deg, 1e-9);
  }
}

void test_false_loops_rejected()
{
  const auto r = make_run();
  const mola::mapper::LoopOdometryCheckParams p;
  // Short range: keyframes 60 m apart along a straight side, matched as if they
  // were 1 m apart (repetitive geometry):
  {
    const auto c = check_loop_against_odometry(r.odo, 20, 80, CPose3D(1.0, 0, 0, 0, 0, 0), p);
    std::cout << "false short loop: " << c.xyz_error << " m off over " << c.distance_traveled
              << " m\n";
    ASSERT_(!c.accepted);
  }
  // One lap later, but matched facing the opposite way:
  {
    const std::size_t i = 40;
    const std::size_t j = i + 300;
    const auto flipped = (r.gt[j] - r.gt[i]) + CPose3D(0, 0, 0, M_PI, 0, 0);
    const auto c = check_loop_against_odometry(r.odo, i, j, flipped, p);
    ASSERT_(!c.accepted);
    ASSERT_GT_(c.ang_error_deg, 170.0);
  }
}

void test_vertical_limit()
{
  // A building with 4 m floors: a keyframe per meter along a 40 m corridor, out
  // and back, on each floor; odometry with 1 % of scale error and 0.2 % of
  // vertical drift.
  std::vector<CPose3D> odo;
  for (int i = 0; i < 160; i++) {
    const int j = i % 80;
    const double z = (i < 80 ? 0.0 : 4.0) + 0.002 * i;
    odo.emplace_back(1.01 * (j < 40 ? j : 80 - j), 0, z, 0, 0, 0);
  }
  // Floor 1 matched to the same spot one floor up (4 m apart), and a true loop:
  const CPose3D sameSpot(0, 0, 0, 0, 0, 0);
  const CPose3D oneUp(0, 0, 4.0, 0, 0, 0);
  mola::mapper::LoopOdometryCheckParams p;
  p.max_xyz_per_m = 0.05;
  const auto off = check_loop_against_odometry(odo, 20, 100, sameSpot, p);
  ASSERT_(off.accepted);  // 4.16 m in 3D is within 2 m + 5 % of ~84 m
  ASSERT_NEAR_(off.z_error, 4.16, 1e-6);

  // The vertical limit, with the default per meter allowance:
  p.max_z = 1.0;
  const auto offZ = check_loop_against_odometry(odo, 20, 100, sameSpot, p);
  std::cout << "stacked floors: " << offZ.z_error << " m vertical over " << offZ.distance_traveled
            << " m\n";
  ASSERT_(!offZ.accepted);
  const auto trueZ = check_loop_against_odometry(odo, 20, 100, oneUp, p);
  ASSERT_(trueZ.accepted);
  ASSERT_LT_(trueZ.z_error, 0.5);

  // The same edge stated the other way around gives the same vertical error:
  const auto offZr = check_loop_against_odometry(odo, 100, 20, -sameSpot, p);
  ASSERT_NEAR_(offZr.z_error, offZ.z_error, 1e-9);
}

void test_vertical_limit_tilted_keyframes()
{
  // A straight 30 deg ramp, a keyframe per meter, exact odometry. The vertical
  // error is measured along z of the odometry frame, not of the tilted keyframe.
  const double a = mrpt::DEG2RAD(30.0);
  std::vector<CPose3D> odo;
  for (int k = 0; k < 100; k++) {
    odo.emplace_back(k * std::cos(a), 0, k * std::sin(a), 0, -a, 0);
  }
  const std::size_t i = 10;
  const std::size_t j = 40;
  mola::mapper::LoopOdometryCheckParams p;
  p.max_z = 1.0;

  const auto ok = check_loop_against_odometry(odo, i, j, odo[j] - odo[i], p);
  ASSERT_(ok.accepted);
  ASSERT_LT_(ok.z_error, 1e-6);

  // The last keyframe displaced 4 m up, or 3 m horizontally, in the odometry frame:
  const auto & last = odo[j];
  const CPose3D up(last.x(), last.y(), last.z() + 4.0, last.yaw(), last.pitch(), last.roll());
  const CPose3D side(last.x() + 3.0, last.y(), last.z(), last.yaw(), last.pitch(), last.roll());

  const auto cUp = check_loop_against_odometry(odo, i, j, up - odo[i], p);
  ASSERT_NEAR_(cUp.z_error, 4.0, 1e-6);
  ASSERT_(!cUp.accepted);

  const auto cSide = check_loop_against_odometry(odo, i, j, side - odo[i], p);
  ASSERT_LT_(cSide.z_error, 1e-6);
  ASSERT_NEAR_(cSide.xyz_error, 3.0, 1e-6);
  ASSERT_(cSide.accepted);
}

}  // namespace

int main()
{
  try {
    test_true_loops_accepted();
    test_false_loops_rejected();
    test_vertical_limit();
    test_vertical_limit_tilted_keyframes();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
