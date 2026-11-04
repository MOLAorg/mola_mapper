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

}  // namespace

int main()
{
  try {
    test_true_loops_accepted();
    test_false_loops_rejected();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
