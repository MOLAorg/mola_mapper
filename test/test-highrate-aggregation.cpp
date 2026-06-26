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
 * @file   test-highrate-aggregation.cpp
 * @brief  High-rate IMU/wheels aggregation: bounded keyframe growth + correct
 *         wheel-chained trajectory (plan section 4.12).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Feeds a synthetic high-rate IMU + wheel-odometry stream (no LIO) and checks:
 *  - with aggregate_high_rate_into_edges=true, the central graph holds
 *    O(duration / sensor_keyframe_min_period) keyframes, NOT O(#samples), and
 *  - the wheel relative-chaining recovers the straight-line trajectory, and
 *  - aggregation OFF produces FAR more keyframes for the same input.
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/Lie/SE.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;  // _deg

namespace
{
std::string params_yaml(bool aggregate)
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
  imu_normalized_gravity_alignment_sigma: 0.5
  sensor_keyframe_min_period: 0.5
  imu_max_insert_rate_hz: 0.0
  odometry_max_insert_rate_hz: 0.0
  aggregate_high_rate_into_edges: )###") +
         (aggregate ? "true" : "false") + "\n";
}

// Feed a straight-line trajectory at `v` m/s along +x for `duration` s, both
// wheel odometry and a level IMU, at `rate` Hz. Returns the number of samples.
int feed_imu_and_wheels(mola::mapper_3d::Mapper3D & nav, double v, double duration, double rate)
{
  const double dt = 1.0 / rate;
  int n = 0;
  for (double t = 0; t <= duration + 1e-9; t += dt) {
    const auto stamp = mrpt::Clock::fromDouble(t);

    mrpt::obs::CObservationOdometry obsOdo;
    obsOdo.timestamp = stamp;
    obsOdo.odometry = mrpt::poses::CPose2D(v * t, 0.0, 0.0);
    nav.fuse_odometry(obsOdo, "odom_wheels");

    mrpt::obs::CObservationIMU obsImu;
    obsImu.timestamp = stamp;
    obsImu.sensorLabel = "imu";
    obsImu.set(mrpt::obs::IMU_X_ACC, 0.0);
    obsImu.set(mrpt::obs::IMU_Y_ACC, 0.0);
    obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81);
    nav.fuse_imu(obsImu);

    n++;
  }
  return n;
}

// Aggregation bounds the keyframe count and recovers the trajectory.
void test_aggregation_bounds_keyframes_and_tracks()
{
  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

  const double v = 1.5;        // m/s
  const double duration = 4.0;  // s
  const double rate = 50.0;     // Hz
  const int nSamples = feed_imu_and_wheels(nav, v, duration, rate);

  // Keyframes should be ~ duration / sensor_keyframe_min_period (= 4/0.5 = 8),
  // i.e. a small constant, NOT proportional to nSamples (~200).
  const std::size_t kfCount = nav.keyframe_count();
  std::cout << "  aggregate=ON: samples=" << nSamples << " keyframes=" << kfCount << "\n";
  ASSERT_LT_(kfCount, static_cast<std::size_t>(20));
  ASSERT_GT_(kfCount, static_cast<std::size_t>(3));

  // No {odom_wheels} frame variable is created in aggregation mode (wheels are
  // pure relative edges between keyframes):
  ASSERT_EQUAL_(nav.known_odometry_frame_ids().count("odom_wheels"), 0U);

  // Trajectory: query near the end; x should match v*t from the wheel chaining.
  const double queryT = duration - 0.05;
  const auto st = nav.estimated_navstate(mrpt::Clock::fromDouble(queryT), "map");
  ASSERT_(st.has_value());
  std::cout << "  estimated x=" << st->pose.mean.x() << " expected~" << v * queryT << "\n";
  ASSERT_NEAR_(st->pose.mean.x(), v * queryT, 0.25);
  // Level (IMU gravity keeps roll/pitch ~0):
  ASSERT_NEAR_(st->pose.mean.z(), 0.0, 0.2);
  ASSERT_LT_(std::abs(mrpt::RAD2DEG(st->pose.mean.pitch())), 3.0);
  ASSERT_LT_(std::abs(mrpt::RAD2DEG(st->pose.mean.roll())), 3.0);
}

// The same input WITHOUT aggregation produces far more keyframes (one per
// sample), confirming aggregation is what bounds the growth.
void test_no_aggregation_explodes()
{
  mola::mapper_3d::Mapper3D navOff;
  navOff.initialize(mrpt::containers::yaml::FromText(params_yaml(false)));
  const int nSamples = feed_imu_and_wheels(navOff, 1.5, 4.0, 50.0);
  // Force a solve so the keyframes are materialized:
  (void)navOff.estimated_navstate(mrpt::Clock::fromDouble(3.95), "map");
  const std::size_t kfOff = navOff.keyframe_count();

  mola::mapper_3d::Mapper3D navOn;
  navOn.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));
  feed_imu_and_wheels(navOn, 1.5, 4.0, 50.0);
  const std::size_t kfOn = navOn.keyframe_count();

  std::cout << "  samples=" << nSamples << " keyframes off=" << kfOff << " on=" << kfOn << "\n";
  // Off creates many more keyframes than on (at least 4x here):
  ASSERT_GT_(kfOff, kfOn * 4);
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_aggregation_bounds_keyframes_and_tracks", test_aggregation_bounds_keyframes_and_tracks},
    {"test_no_aggregation_explodes", test_no_aggregation_explodes},
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
