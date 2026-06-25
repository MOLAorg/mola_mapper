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
 * @file   test-navstate-fusion.cpp
 * @brief  Phase 1 fusion tests: single-frame extrapolation + two-odometry fusion.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Adapted from mola_state_estimation_smoother's test-navstate-basic.cpp and
 * test-two-odometries.cpp, since Mapper3D's NavStateFilter fusion mirrors the
 * smoother (native iSAM2 graph over the central keyframes).
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;  // _deg

namespace
{
const char * kParams =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.10
  initial_twist_sigma_lin: 20.0
  initial_twist_sigma_ang: 3.0
)###";

const char * kParamsTwoOdom =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  link_first_pose_to_reference_origin_sigma: 1e-6
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 2.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.10
)###";

mrpt::poses::CPose3DPDFGaussian pdf_at(
  double x, double y = 0, double z = 0, double yaw_deg = 0, double sigma = 0.02)
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, z, mrpt::DEG2RAD(yaw_deg), 0.0_deg, 0.0_deg);
  p.cov.setIdentity();
  p.cov *= sigma;
  return p;
}

double pose_err(const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b)
{
  return mrpt::poses::Lie::SE<3>::log(a - b).norm();
}

void test_one_pose()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParams));

  const auto t0 = mrpt::Clock::fromDouble(0.0);
  const auto p0 = pdf_at(0.0);
  nav.fuse_pose(t0, p0, "odom");

  const auto ret = nav.estimated_navstate(t0, "odom");
  ASSERT_(ret.has_value());
  ASSERT_NEAR_(pose_err(ret->pose.mean, p0.mean), 0.0, 1e-2);
}

void test_two_poses_extrapolate()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParams));

  nav.fuse_pose(mrpt::Clock::fromDouble(0.0), pdf_at(0.0), "odom");
  nav.fuse_pose(mrpt::Clock::fromDouble(0.5), pdf_at(0.5), "odom");

  // Constant-velocity extrapolation to t=0.6 -> x ~ 0.6
  const auto ret = nav.estimated_navstate(mrpt::Clock::fromDouble(0.6), "odom");
  ASSERT_(ret.has_value());
  const auto expected = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.6, 0, 0, 0, 0, 0);
  ASSERT_NEAR_(pose_err(ret->pose.mean, expected), 0.0, 5e-2);
}

void test_velocity_estimate_straight()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParams));

  auto & rng = mrpt::random::getRandomGenerator();
  const double vx = 8.0;  // m/s
  const double T = 0.1;
  const size_t nSteps = 10;
  for (size_t i = 0; i < nSteps; i++) {
    const double tt = T * static_cast<double>(i);
    nav.fuse_pose(
      mrpt::Clock::fromDouble(tt), pdf_at(vx * tt + rng.drawGaussian1D(0, 0.01)), "odom");
  }

  const auto ret = nav.estimated_navstate(mrpt::Clock::fromDouble(T * nSteps), "odom");
  ASSERT_(ret.has_value());
  ASSERT_NEAR_(ret->twist.vx, vx, 0.5);
  ASSERT_NEAR_(ret->twist.vy, 0.0, 0.5);
  ASSERT_NEAR_(ret->twist.wz, 0.0, 0.1);
}

void test_two_odometries()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(kParamsTwoOdom));

  auto & rng = mrpt::random::getRandomGenerator();

  const double T = 0.1;
  const size_t numPoses = 300;
  const auto gtDelta = mrpt::poses::CPose3D(1.0 * T, 0, 0, 0.2 * T, 0, 0);

  mrpt::poses::CPose3D gtPose = mrpt::poses::CPose3D::Identity();
  // Wheels start at a different (known to the graph only via fusion) offset:
  mrpt::poses::CPose3D odomWheels =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(-20.0, 10.0, 0.0, 42.0_deg, 0.0_deg, 0.0_deg);
  mrpt::poses::CPose3D odomLidar = mrpt::poses::CPose3D::Identity();

  double sumErrWheels = 0;
  double sumErrFused = 0;
  size_t nEval = 0;

  for (size_t i = 0; i < numPoses; i++) {
    const auto stamp = mrpt::Clock::fromDouble(T * static_cast<double>(i));
    if (i > 0) {
      gtPose = gtPose + gtDelta;
    }

    // Wheels: 2% scale drift in X + noise.
    auto deltaWheels = gtDelta;
    deltaWheels.x(deltaWheels.x() * 1.02);
    deltaWheels.x_incr(rng.drawGaussian1D(0, 0.05));
    deltaWheels.y_incr(rng.drawGaussian1D(0, 0.05));
    odomWheels = odomWheels + deltaWheels;
    mrpt::poses::CPose3DPDFGaussian wheelsPdf;
    wheelsPdf.mean = odomWheels;
    wheelsPdf.cov.setIdentity();
    wheelsPdf.cov *= mrpt::square(0.05);

    // Lidar: accurate, small sideways drift.
    auto deltaLidar = gtDelta;
    deltaLidar.y_incr(0.005);
    deltaLidar.x_incr(rng.drawGaussian1D(0, 0.01));
    deltaLidar.y_incr(rng.drawGaussian1D(0, 0.01));
    odomLidar = odomLidar + deltaLidar;
    mrpt::poses::CPose3DPDFGaussian lidarPdf;
    lidarPdf.mean = odomLidar;
    lidarPdf.cov.setIdentity();
    lidarPdf.cov *= mrpt::square(0.01);

    nav.fuse_pose(stamp, wheelsPdf, "wheels_odom");
    nav.fuse_pose(stamp, lidarPdf, "lidar_odom");

    if (i > 50 && (i % 10) == 0) {
      const auto st = nav.estimated_navstate(stamp, "map");
      ASSERT_(st.has_value());
      sumErrWheels += pose_err(odomWheels, gtPose);
      sumErrFused += pose_err(st->pose.mean, gtPose);
      nEval++;
    }
  }

  ASSERT_(nEval > 0);
  std::cout << "  mean err wheels=" << sumErrWheels / static_cast<double>(nEval)
            << "  fused=" << sumErrFused / static_cast<double>(nEval) << "\n";
  // Fusion (anchored to the map origin + accurate lidar) must beat raw wheels:
  ASSERT_LT_(sumErrFused, sumErrWheels);

  // Both odometry frames must be known and have an estimated transform:
  ASSERT_EQUAL_(nav.known_odometry_frame_ids().size(), 2U);
  for (const auto & f : nav.known_odometry_frame_ids()) {
    ASSERT_(nav.estimated_T_map_to_odometry_frame(f).has_value());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_one_pose", test_one_pose},
    {"test_two_poses_extrapolate", test_two_poses_extrapolate},
    {"test_velocity_estimate_straight", test_velocity_estimate_straight},
    {"test_two_odometries", test_two_odometries},
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
