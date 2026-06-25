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
 * @file   test-optimizer-thread.cpp
 * @brief  Background optimizer thread: async queries return the correct fused
 *         estimate, and the synchronous and threaded paths agree.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * The threaded path (enable_optimizer_thread: true) runs the iSAM2 solve off
 * the query path, so a query right after fusing may read a one-cycle-stale
 * cache. This test therefore polls (bounded) until the backend catches up, then
 * checks the result matches what the deterministic synchronous path produces.
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <thread>

using namespace mrpt::literals;  // _deg

namespace
{
std::string params_yaml(bool threaded)
{
  return std::string(R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  link_first_pose_to_reference_origin_sigma: 1e-6
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  enable_optimizer_thread: )###") +
         (threaded ? "true" : "false") + "\n";
}

mrpt::poses::CPose3DPDFGaussian pdf_at(double x, double sigma = 0.02)
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
  p.cov.setIdentity();
  p.cov *= sigma;
  return p;
}

double pose_err(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b)
{
  return mrpt::poses::Lie::SE<3>::log(a - b).norm();
}

// Feed a straight-line, constant-velocity trajectory at 1 m/s along +x.
void feed_straight_line(mola::mapper_3d::Mapper3D & nav, int steps, double dt)
{
  for (int i = 0; i <= steps; i++) {
    const double t = i * dt;
    nav.fuse_pose(mrpt::Clock::fromDouble(t), pdf_at(t), "odom");
  }
}

// Poll estimated_navstate() until the x estimate reaches `expectedX` (the
// backend thread caught up) or the timeout elapses.
std::optional<mola::NavState> poll_until_converged(
  mola::mapper_3d::Mapper3D & nav, double queryT, double expectedX, double tol, double timeout_s)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::optional<mola::NavState> last;
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
    last = nav.estimated_navstate(mrpt::Clock::fromDouble(queryT), "odom");
    if (last.has_value() && std::abs(last->pose.mean.x() - expectedX) < tol) {
      return last;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return last;
}

// The background optimizer thread eventually reflects all fused data, and the
// estimate matches the known constant-velocity trajectory.
void test_threaded_query_converges()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

  const int steps = 50;
  const double dt = 0.1;  // 1 m/s along x over 5 s
  feed_straight_line(nav, steps, dt);

  const double lastT = steps * dt;  // 5.0 s -> x ~ 5.0 m
  const auto ret = poll_until_converged(nav, lastT, lastT, 0.1, 5.0);
  ASSERT_(ret.has_value());
  const auto expected = mrpt::poses::CPose3D::FromXYZYawPitchRoll(lastT, 0, 0, 0, 0, 0);
  ASSERT_NEAR_(pose_err(ret->pose.mean, expected), 0.0, 0.1);

  // Constant-velocity extrapolation past the last keyframe must also work from
  // the cached anchor (no fresh solve on the query path):
  const auto extra = nav.estimated_navstate(mrpt::Clock::fromDouble(lastT + 0.2), "odom");
  ASSERT_(extra.has_value());
  const auto expectedExtra = mrpt::poses::CPose3D::FromXYZYawPitchRoll(lastT + 0.2, 0, 0, 0, 0, 0);
  ASSERT_NEAR_(pose_err(extra->pose.mean, expectedExtra), 0.0, 0.15);
}

// The threaded and synchronous paths converge to the same estimate.
void test_threaded_matches_synchronous()
{
  mola::mapper_3d::Mapper3D navSync;
  navSync.initialize(mrpt::containers::yaml::FromText(params_yaml(false)));
  mola::mapper_3d::Mapper3D navThreaded;
  navThreaded.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

  const int steps = 40;
  const double dt = 0.1;
  feed_straight_line(navSync, steps, dt);
  feed_straight_line(navThreaded, steps, dt);

  const double lastT = steps * dt;

  // Synchronous path is immediately fresh:
  const auto retSync = navSync.estimated_navstate(mrpt::Clock::fromDouble(lastT), "odom");
  ASSERT_(retSync.has_value());

  // Threaded path: poll until caught up:
  const auto retThreaded = poll_until_converged(navThreaded, lastT, retSync->pose.mean.x(), 0.05, 5.0);
  ASSERT_(retThreaded.has_value());

  ASSERT_NEAR_(pose_err(retSync->pose.mean, retThreaded->pose.mean), 0.0, 0.05);
}

// Destroying the module while the optimizer thread is running must join cleanly
// (no crash / no hang). Exercised implicitly here by going out of scope right
// after enqueuing a burst of work.
void test_clean_shutdown_while_busy()
{
  for (int rep = 0; rep < 3; rep++) {
    mola::mapper_3d::Mapper3D nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));
    feed_straight_line(nav, 30, 0.1);
    // Intentionally do NOT wait: the destructor must stop+join the thread even
    // with work still pending.
  }
  // Reaching here without deadlock/crash is the assertion.
  ASSERT_(true);
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_threaded_query_converges", test_threaded_query_converges},
    {"test_threaded_matches_synchronous", test_threaded_matches_synchronous},
    {"test_clean_shutdown_while_busy", test_clean_shutdown_while_busy},
  };

  int runOnlyIdx = -1;
  if (argc == 2) {
    runOnlyIdx = std::stoi(argv[1]);
  }

  bool anyFail = false;
  int index = 0;
  for (const auto & [name, f] : tests) {
    index++;
    if (runOnlyIdx >= 0 && index != runOnlyIdx) {
      continue;
    }
    try {
      std::cout << "[ " << index << " / " << tests.size() << " ] " << name << " ..." << std::endl;
      f();
      std::cout << "   OK." << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "   ERROR: " << mrpt::exception_to_str(e) << std::endl;
      anyFail = true;
    }
  }
  return anyFail ? 1 : 0;
}
