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
 * @file   test-mapper3d-basic.cpp
 * @brief  Phase 1 skeleton tests: lifecycle, frame registry, keyframe guard.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>

#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;

namespace
{
const char * kParams =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.05
)###";

mrpt::poses::CPose3DPDFGaussian make_pdf(double x)
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
  p.cov.setIdentity();
  p.cov *= 0.02;
  return p;
}

void test_lifecycle_and_frames()
{
  mola::mapper::Mapper m;
  m.initialize(mrpt::containers::yaml::FromText(kParams));

  ASSERT_(m.known_odometry_frame_ids().empty());
  ASSERT_EQUAL_(m.keyframe_count(), 0U);

  // Skeleton: estimated_navstate not implemented yet -> nullopt.
  ASSERT_(!m.estimated_navstate(mrpt::Clock::fromDouble(0.0), "odom").has_value());

  const auto t0 = mrpt::Clock::fromDouble(0.0);
  m.fuse_pose(t0, make_pdf(0.0), "odom");

  ASSERT_EQUAL_(m.keyframe_count(), 1U);
  ASSERT_EQUAL_(m.known_odometry_frame_ids().size(), 1U);
  ASSERT_(m.known_odometry_frame_ids().count("odom") == 1);
}

void test_keyframe_creation_and_reuse()
{
  mola::mapper::Mapper m;
  m.initialize(mrpt::containers::yaml::FromText(kParams));

  // Two well-separated timestamps -> two keyframes.
  m.fuse_pose(mrpt::Clock::fromDouble(0.0), make_pdf(0.0), "odom");
  m.fuse_pose(mrpt::Clock::fromDouble(0.3), make_pdf(0.3), "odom");
  ASSERT_EQUAL_(m.keyframe_count(), 2U);

  // A timestamp within min_time_difference of an existing KF -> reuse.
  m.fuse_pose(mrpt::Clock::fromDouble(0.31), make_pdf(0.31), "odom");
  ASSERT_EQUAL_(m.keyframe_count(), 2U);
}

void test_out_of_order_guard()
{
  mola::mapper::Mapper m;
  m.initialize(mrpt::containers::yaml::FromText(kParams));

  m.fuse_pose(mrpt::Clock::fromDouble(0.0), make_pdf(0.0), "odom");
  m.fuse_pose(mrpt::Clock::fromDouble(0.3), make_pdf(0.3), "odom");
  ASSERT_EQUAL_(m.keyframe_count(), 2U);

  // A request at t=0.15 is older than the newest KF (0.3) and NOT within
  // min_time_difference (0.05) of any existing KF: the out-of-order guard must
  // snap it to the nearest existing KF instead of inserting a past variable.
  m.fuse_pose(mrpt::Clock::fromDouble(0.15), make_pdf(0.15), "odom");
  ASSERT_EQUAL_(m.keyframe_count(), 2U);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_lifecycle_and_frames", test_lifecycle_and_frames},
    {"test_keyframe_creation_and_reuse", test_keyframe_creation_and_reuse},
    {"test_out_of_order_guard", test_out_of_order_guard},
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
