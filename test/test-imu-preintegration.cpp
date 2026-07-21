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
 * @file   test-imu-preintegration.cpp
 * @brief  Exercises the between-keyframe IMU preintegration (CombinedImuFactor).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * The RELATIVE half of IMU fusion adds a gtsam CombinedImuFactor per keyframe
 * transition (world velocity Vw + bias B in-graph). This test drives a moving
 * trajectory with a tilted IMU mount and preintegration ENABLED, and checks
 * that (a) iSAM2 stays stable with the new factors/variables in the loop, and
 * (b) the map still levels (the absolute gravity anchors + the relative chain
 * coexist without fighting). A full "level-while-continuously-moving without any
 * static anchor" observability test is a follow-up.
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <string>

using namespace mrpt::literals;

namespace
{
void run_preintegration_leveling(const mrpt::poses::CPose3D & imuMountPose)
{
  const std::string params =
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
  imu_gravity_min_samples: 1
  link_first_pose_to_reference_origin_sigma: 0.01
  imu_attitude_sigma_deg: 2.0
  # The RELATIVE half under test:
  imu_preintegration_enabled: true
  imu_preint_accel_noise_sigma: 0.05
  imu_preint_gyro_noise_sigma: 0.005
)###";

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  auto & rng = mrpt::random::getRandomGenerator();

  const size_t numSteps = 100;
  const double T = 0.05;

  // A tilted LiDAR-odometry frame; the accelerometer should still level it.
  mrpt::poses::CPose3D currentOdom =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, 3.0_deg, 2.0_deg);

  for (size_t i = 1; i <= numSteps; i++) {
    const auto time = mrpt::Clock::fromDouble(T * static_cast<double>(i));

    auto deltaOdom = mrpt::poses::CPose3D(0.5 * T, 0, 0, 0.1 * T, 0, 0);
    deltaOdom.z_incr(-0.002);
    currentOdom = currentOdom + deltaOdom;

    mrpt::poses::CPose3DPDFGaussian odomLidarPdf;
    odomLidarPdf.mean = currentOdom;
    odomLidarPdf.cov.setIdentity();
    odomLidarPdf.cov *= 1e-3;

    mrpt::obs::CObservationIMU obsImu;
    obsImu.timestamp = time;
    obsImu.sensorLabel = "imu";
    obsImu.sensorPose = imuMountPose;

    // Physically flat robot: gravity straight down in the vehicle frame, plus a
    // small body acceleration; measured in the rotated sensor frame.
    const auto aVehicle = mrpt::math::TVector3D(
      rng.drawGaussian1D(0.0, 0.1), rng.drawGaussian1D(0.0, 0.1),
      9.81 + rng.drawGaussian1D(0.0, 0.1));
    const auto aSensor = imuMountPose.inverseRotateVector(aVehicle);
    obsImu.set(mrpt::obs::IMU_X_ACC, aSensor.x);
    obsImu.set(mrpt::obs::IMU_Y_ACC, aSensor.y);
    obsImu.set(mrpt::obs::IMU_Z_ACC, aSensor.z);

    // Small yaw rate (matches the trajectory), expressed in the sensor frame.
    const auto wVehicle = mrpt::math::TVector3D(0, 0, 0.1);
    const auto wSensor = imuMountPose.inverseRotateVector(wVehicle);
    obsImu.set(mrpt::obs::IMU_WX, wSensor.x);
    obsImu.set(mrpt::obs::IMU_WY, wSensor.y);
    obsImu.set(mrpt::obs::IMU_WZ, wSensor.z);

    nav.fuse_imu(obsImu);
    nav.fuse_pose(time, odomLidarPdf, "lidar");
  }

  const auto stateOpt =
    nav.estimated_navstate(mrpt::Clock::fromDouble(T * static_cast<double>(numSteps)), "map");
  ASSERT_(stateOpt.has_value());

  double y = 0;
  double p = 0;
  double r = 0;
  stateOpt->pose.mean.getYawPitchRoll(y, p, r);
  std::cout << "  [preint] leveled pitch=" << mrpt::RAD2DEG(p) << " roll=" << mrpt::RAD2DEG(r)
            << " deg\n";

  // With preintegration ON, the absolute gravity anchors must still level the
  // map (the relative chain must not fight them into a tilt).
  ASSERT_NEAR_(p, 0.0, mrpt::DEG2RAD(5.0));
  ASSERT_NEAR_(r, 0.0, mrpt::DEG2RAD(5.0));
}
}  // namespace

int main()
{
  try {
    std::cout << "-- preintegration, identity mount --\n";
    run_preintegration_leveling(mrpt::poses::CPose3D::Identity());

    std::cout << "-- preintegration, generic tilted mount --\n";
    run_preintegration_leveling(
      mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.3, -0.1, 0.5, 90.0_deg, -30.0_deg, 45.0_deg));

    std::cout << "Test successful." << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
