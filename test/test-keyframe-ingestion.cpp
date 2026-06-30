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
 * @file   test-keyframe-ingestion.cpp
 * @brief  Phase 3 tests: SharedKeyframeMap sink, synthetic LIO keyframe stream.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>

using namespace mrpt::literals;

namespace
{
mola::SharedKeyframeMap::KeyframeInsertRequest make_request(
  const mrpt::Clock::time_point & t, const std::string & sourceId,
  const mrpt::poses::CPose3D & poseInSource, double sigmaLin, double sigmaAng)
{
  mola::SharedKeyframeMap::KeyframeInsertRequest req;
  req.timestamp = t;
  req.source_frame_id = sourceId;
  req.pose_in_source.mean = poseInSource;
  req.pose_in_source.cov.setIdentity();
  for (int i = 0; i < 3; i++) {
    req.pose_in_source.cov(i, i) = mrpt::square(sigmaLin);
  }
  for (int i = 3; i < 6; i++) {
    req.pose_in_source.cov(i, i) = mrpt::square(sigmaAng);
  }
  req.quality = 1.0;
  return req;
}

// ---------------------------------------------------------------------------
// Basic plumbing: a clean (no-drift) synthetic LIO keyframe stream is
// ingested via requestInsertKeyframe() and recovered close to ground truth.
// ---------------------------------------------------------------------------
void test_keyframe_ingestion_basic()
{
  const char * params =
    R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  imu_max_insert_rate_hz: 0.0
  odometry_max_insert_rate_hz: 0.0
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.05
  sigma_integrator_orientation: 0.05
  keyframe_ingestion_sigma_lin: 0.01
  keyframe_ingestion_sigma_ang_deg: 0.2
)###";

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  constexpr size_t numKFs = 10;
  constexpr double T = 0.1;
  constexpr double v = 1.0;  // [m/s]

  mrpt::poses::CPose3D currentOdom = mrpt::poses::CPose3D::Identity();

  for (size_t i = 1; i <= numKFs; i++) {
    const auto t = mrpt::Clock::fromDouble(T * static_cast<double>(i));
    currentOdom = currentOdom + mrpt::poses::CPose3D(v * T, 0, 0, 0.0, 0.0, 0.0);

    const auto req = make_request(t, "odom_lidar", currentOdom, 0.01, 0.2);
    const auto kfId = nav.requestInsertKeyframe(req);
    ASSERT_(kfId.has_value());
  }

  ASSERT_EQUAL_(nav.keyframe_count(), numKFs);

  const auto stateOpt =
    nav.estimated_navstate(mrpt::Clock::fromDouble(T * static_cast<double>(numKFs)), "map");
  ASSERT_(stateOpt.has_value());

  const double expectedX = v * T * static_cast<double>(numKFs);
  std::cout << "  final x=" << stateOpt->pose.mean.x() << " expected=" << expectedX << "\n";
  ASSERT_NEAR_(stateOpt->pose.mean.x(), expectedX, 0.05);
}

// ---------------------------------------------------------------------------
// A synthetic LIO instance accumulates a growing pitch bias in its own
// odometry frame (a per-step systematic ICP rotation error: the realistic
// long-term LIO z/tilt drift failure mode). Its OWN reported absolute pose
// drifts away from level/flat, dragging an apparent z drift with it (forward
// motion projected through an increasingly tilted frame as poses are
// composed). The keyframe-ingestion sink only trusts the *consecutive*
// relative motion, so chaining alone reproduces the
// per-step LOCAL motion faithfully (zero relative z, just the tiny per-step
// pitch bias) without baking in that absolute-composition artifact; IMU
// gravity leveling on each keyframe then pulls the *absolute* orientation
// back to level, correcting the drift vs ground truth.
// ---------------------------------------------------------------------------
void test_keyframe_ingestion_imu_corrects_drift()
{
  const char * params =
    R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  odometry_max_insert_rate_hz: 0.0
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  sigma_random_walk_acceleration_linear: 2.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.5
  # Dense (per-step) keyframes => ~1 IMU sample per interval; allow the gravity
  # reducer to emit from a single sample (real LIO keyframes are sparse).
  imu_gravity_min_samples: 1
  keyframe_ingestion_sigma_lin: 0.02
  keyframe_ingestion_sigma_ang_deg: 0.5
  link_first_pose_to_reference_origin_sigma: 0.01
)###";

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  auto & rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);

  constexpr size_t numSteps = 100;
  constexpr double T = 0.05;
  constexpr double v = 0.5;                              // [m/s] forward speed
  const double pitchDriftPerStep = mrpt::DEG2RAD(0.04);  // per-step systematic ICP bias

  // The synthetic LIO's own (drifting) absolute pose, in its own odom frame:
  mrpt::poses::CPose3D currentOdom = mrpt::poses::CPose3D::Identity();

  // Mid-trajectory checkpoints to verify local shape (relative motion) is
  // preserved despite the absolute drift / its correction:
  constexpr size_t kMid1Step = 40;
  constexpr size_t kMid2Step = 70;
  std::optional<mrpt::Clock::time_point> tMid1;
  std::optional<mrpt::Clock::time_point> tMid2;

  for (size_t i = 1; i <= numSteps; i++) {
    const auto t = mrpt::Clock::fromDouble(T * static_cast<double>(i));

    // Local-frame relative motion: pure forward step plus a tiny, systematic
    // pitch bias (zero relative z: the apparent z drift only emerges once
    // this local delta is composed through the already-tilted accumulated
    // orientation, i.e. it is an artifact of absolute composition, not of
    // the local measurement itself).
    const mrpt::poses::CPose3D deltaOdom(v * T, 0, 0, 0.0, pitchDriftPerStep, 0.0);
    currentOdom = currentOdom + deltaOdom;

    // IMU: the robot is physically flat; the accelerometer senses gravity
    // straight down in the body frame regardless of LIO's drifted estimate.
    // Streamed in BEFORE the keyframe request, so the sample is already
    // buffered when the keyframe closes and drains its interval.
    mrpt::obs::CObservationIMU obsImu;
    obsImu.timestamp = t;
    obsImu.sensorLabel = "imu";
    obsImu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0.0, 0.05));
    obsImu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0.0, 0.05));
    obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0.0, 0.05));
    nav.fuse_imu(obsImu);

    const auto req = make_request(t, "odom_lidar", currentOdom, 0.01, 0.3);
    const auto kfId = nav.requestInsertKeyframe(req);
    ASSERT_(kfId.has_value());

    if (i == kMid1Step) {
      tMid1 = t;
    }
    if (i == kMid2Step) {
      tMid2 = t;
    }
  }

  ASSERT_EQUAL_(nav.keyframe_count(), numSteps);

  // Raw LIO drift (what the absolute, uncorrected odometry alone reports):
  double rawYaw = 0;
  double rawPitch = 0;
  double rawRoll = 0;
  currentOdom.getYawPitchRoll(rawYaw, rawPitch, rawRoll);
  std::cout << "  raw LIO drift: pitch=" << mrpt::RAD2DEG(rawPitch) << " deg  z=" << currentOdom.z()
            << " m\n";

  // Corrected (mapper_3d's optimized) final state:
  const auto stateOpt =
    nav.estimated_navstate(mrpt::Clock::fromDouble(T * static_cast<double>(numSteps)), "map");
  ASSERT_(stateOpt.has_value());

  double corrYaw = 0;
  double corrPitch = 0;
  double corrRoll = 0;
  stateOpt->pose.mean.getYawPitchRoll(corrYaw, corrPitch, corrRoll);
  std::cout << "  corrected: pitch=" << mrpt::RAD2DEG(corrPitch)
            << " deg  z=" << stateOpt->pose.mean.z() << " m\n";

  // The correction must be substantially better than the raw drift, and
  // close to the (flat, zero-tilt) ground truth:
  ASSERT_LT_(std::abs(corrPitch), std::abs(rawPitch));
  ASSERT_NEAR_(corrPitch, 0.0, mrpt::DEG2RAD(1.5));
  ASSERT_NEAR_(stateOpt->pose.mean.z(), 0.0, 0.05);

  // Local shape preservation: the mid-trajectory along-track distance must
  // match ground truth regardless of the global tilt correction (2.8: the
  // consecutive-frame chain preserves local geometry).
  ASSERT_(tMid1.has_value() && tMid2.has_value());
  const auto st1 = nav.estimated_navstate(*tMid1, "map");
  const auto st2 = nav.estimated_navstate(*tMid2, "map");
  ASSERT_(st1.has_value() && st2.has_value());

  const double dx = st2->pose.mean.x() - st1->pose.mean.x();
  const double expectedDx = v * T * static_cast<double>(kMid2Step - kMid1Step);
  std::cout << "  mid-trajectory dx=" << dx << " expected=" << expectedDx << "\n";
  ASSERT_NEAR_(dx, expectedDx, 0.05);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_keyframe_ingestion_basic", test_keyframe_ingestion_basic},
    {"test_keyframe_ingestion_imu_corrects_drift", test_keyframe_ingestion_imu_corrects_drift},
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
