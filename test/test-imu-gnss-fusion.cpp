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
 * @file   test-imu-gnss-fusion.cpp
 * @brief  Phase 2 tests: IMU gravity leveling, static GNSS+IMU geo-ref, twist.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Adapted from mola_state_estimation_smoother's test-lidar-imu-leveling.cpp and
 * test-static-gnss-imu-orientation.cpp.
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;

namespace
{
// ---------------------------------------------------------------------------
// IMU gravity leveling: a tilted LiDAR-odometry frame is leveled by the IMU
// accelerometer (robot is physically flat).
// ---------------------------------------------------------------------------
void test_imu_leveling()
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
  sigma_random_walk_acceleration_linear: 2.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.5
  imu_normalized_gravity_alignment_sigma: 0.1
  link_first_pose_to_reference_origin_sigma: 0.01
)###";

  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  auto & rng = mrpt::random::getRandomGenerator();

  const size_t numSteps = 100;
  const double T = 0.05;

  // The LiDAR-odometry frame starts with a bad pitch/roll error:
  mrpt::poses::CPose3D currentOdom =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, 3.0_deg, 2.0_deg);

  for (size_t i = 1; i <= numSteps; i++) {
    const auto time = mrpt::Clock::fromDouble(T * static_cast<double>(i));

    auto deltaOdom = mrpt::poses::CPose3D(0.5 * T, 0, 0, 0.1 * T, 0, 0);
    deltaOdom.z_incr(-0.002);
    deltaOdom.setYawPitchRoll(deltaOdom.yaw(), 0.1_deg, deltaOdom.roll());
    currentOdom = currentOdom + deltaOdom;

    mrpt::poses::CPose3DPDFGaussian odomLidarPdf;
    odomLidarPdf.mean = currentOdom;
    odomLidarPdf.cov.setIdentity();
    odomLidarPdf.cov *= 1e-3;

    mrpt::obs::CObservationIMU obsImu;
    obsImu.timestamp = time;
    obsImu.sensorLabel = "imu";
    // Robot is physically flat: gravity ~ straight down in sensor frame.
    obsImu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0.0, 0.1));
    obsImu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0.0, 0.1));
    obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0.0, 0.1));

    nav.fuse_pose(time, odomLidarPdf, "lidar");
    nav.fuse_imu(obsImu);
  }

  const auto stateOpt =
    nav.estimated_navstate(mrpt::Clock::fromDouble(T * static_cast<double>(numSteps)), "map");
  ASSERT_(stateOpt.has_value());

  double y = 0;
  double p = 0;
  double r = 0;
  stateOpt->pose.mean.getYawPitchRoll(y, p, r);
  std::cout << "  leveled pitch=" << mrpt::RAD2DEG(p) << " roll=" << mrpt::RAD2DEG(r) << " deg\n";

  ASSERT_NEAR_(p, 0.0, mrpt::DEG2RAD(4.0));
  ASSERT_NEAR_(r, 0.0, mrpt::DEG2RAD(4.0));
}

// ---------------------------------------------------------------------------
// Static vehicle with a fixed geo-reference: GNSS gives position, IMU gives
// orientation. Recover the vehicle pose and check T_enu_to_map is available.
// ---------------------------------------------------------------------------
void run_static_gnss_imu(const mrpt::poses::CPose3D & vehiclePose, double yaw_gt_deg)
{
  const double latDeg = 36.8407;
  const double lonDeg = -2.4093;
  const double altM = 100.0;

  auto cfg = mrpt::containers::yaml::FromText(
    R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  imu_max_insert_rate_hz: 0.0
  odometry_max_insert_rate_hz: 0.0
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.05
  sigma_random_walk_acceleration_linear: 0.1
  sigma_random_walk_acceleration_angular: 0.1
  sigma_integrator_position: 0.02
  sigma_integrator_orientation: 0.02
  estimate_geo_reference: false
)###");
  cfg["params"]["fixed_geo_reference"]["latitude_deg"] = latDeg;
  cfg["params"]["fixed_geo_reference"]["longitude_deg"] = lonDeg;
  cfg["params"]["fixed_geo_reference"]["altitude"] = altM;

  mola::mapper_3d::Mapper3D nav;
  nav.initialize(cfg);

  auto & rng = mrpt::random::getRandomGenerator();

  mrpt::topography::TGeodeticCoords geoReference;
  geoReference.lat = latDeg;
  geoReference.lon = lonDeg;
  geoReference.height = altM;

  // Geodetic coords of the ground-truth vehicle ENU position:
  mrpt::topography::TGeocentricCoords vehicleGeocentric;
  mrpt::topography::ENUToGeocentric(
    {vehiclePose.x(), vehiclePose.y(), vehiclePose.z()}, geoReference, vehicleGeocentric,
    mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
  mrpt::topography::TGeodeticCoords vehicleGeodetic;
  mrpt::topography::geocentricToGeodetic(vehicleGeocentric, vehicleGeodetic);

  // IMU global orientation: vehicle yaw maps to ENU via -90 deg (yaw=0 => East).
  const auto vehicleToAzimuth =
    mrpt::poses::CPose3D::FromYawPitchRoll(mrpt::DEG2RAD(-90.0), 0.0_deg, 0.0_deg);
  const auto imuPoseGlobal = vehiclePose + vehicleToAzimuth;

  constexpr double GNSS_NOISE_XY_M = 0.10;
  constexpr double GNSS_NOISE_Z_M = 0.15;
  const size_t numReadings = 30;
  const double T = 0.2;

  for (size_t i = 0; i < numReadings; i++) {
    const auto stamp = mrpt::Clock::fromDouble(T * static_cast<double>(i));

    // GNSS:
    {
      mrpt::obs::CObservationGPS obsGps;
      obsGps.timestamp = stamp;
      obsGps.sensorLabel = "gnss";
      constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

      mrpt::obs::gnss::Message_NMEA_GGA gga;
      gga.fields.latitude_degrees = vehicleGeodetic.lat + rng.drawGaussian1D(0, gnss_noise_deg);
      gga.fields.longitude_degrees = vehicleGeodetic.lon + rng.drawGaussian1D(0, gnss_noise_deg);
      gga.fields.altitude_meters = vehicleGeodetic.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
      gga.fields.fix_quality = 4;
      obsGps.setMsg(gga);

      auto & cov = obsGps.covariance_enu.emplace();
      cov.setIdentity();
      cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
      cov(2, 2) = mrpt::square(GNSS_NOISE_Z_M);

      nav.fuse_gnss(obsGps);
    }

    // IMU orientation + gravity:
    {
      mrpt::obs::CObservationIMU obsImu;
      obsImu.timestamp = stamp;
      obsImu.sensorLabel = "imu";

      mrpt::math::CQuaternionDouble gtQuat;
      imuPoseGlobal.getAsQuaternion(gtQuat);
      mrpt::math::CQuaternionDouble noisyQuat;
      for (int k = 0; k < 4; k++) {
        noisyQuat[k] = gtQuat[k] + rng.drawGaussian1D(0, 0.01);
      }
      noisyQuat.normalize();
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_W, noisyQuat.w());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_X, noisyQuat.x());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_Y, noisyQuat.y());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_Z, noisyQuat.z());

      const auto localUp = imuPoseGlobal.inverseRotateVector({0, 0, 9.81});
      obsImu.set(mrpt::obs::IMU_X_ACC, localUp.x + rng.drawGaussian1D(0, 0.1));
      obsImu.set(mrpt::obs::IMU_Y_ACC, localUp.y + rng.drawGaussian1D(0, 0.1));
      obsImu.set(mrpt::obs::IMU_Z_ACC, localUp.z + rng.drawGaussian1D(0, 0.1));

      // Exercise the RawDataConsumer dispatch path too:
      nav.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(obsImu));
    }
  }

  const auto stateOpt = nav.estimated_navstate(
    mrpt::Clock::fromDouble(T * static_cast<double>(numReadings - 1)), "map");
  ASSERT_(stateOpt.has_value());

  const auto est = stateOpt->pose.mean;
  const double posError = (est.asTPose() - vehiclePose.asTPose()).norm();
  double yaw_est = 0;
  double pitch_est = 0;
  double roll_est = 0;
  est.getYawPitchRoll(yaw_est, pitch_est, roll_est);
  const double headingError = std::abs(mrpt::math::angDistance(yaw_est, mrpt::DEG2RAD(yaw_gt_deg)));

  std::cout << "  pos error=" << posError << " m  heading error=" << mrpt::RAD2DEG(headingError)
            << " deg\n";
  ASSERT_LT_(posError, 0.5);
  ASSERT_LT_(headingError, mrpt::DEG2RAD(4.0));

  // Geo-referencing must be available (fixed):
  const auto Tenu = nav.estimated_T_enu_to_map();
  ASSERT_(Tenu.has_value());
  ASSERT_(nav.current_georeferencing().has_value());
}

void test_static_gnss_imu()
{
  mrpt::random::getRandomGenerator().randomize(5678);
  run_static_gnss_imu(mrpt::poses::CPose3D::Identity(), 0.0);
  run_static_gnss_imu(
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(10.0, 20.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg), 30.0);
}

// ---------------------------------------------------------------------------
// Twist fusion: a direct twist measurement constrains the keyframe velocity.
// ---------------------------------------------------------------------------
void test_twist_fusion()
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
  link_first_pose_to_reference_origin_sigma: 1e-3
  initial_twist_sigma_lin: 20.0
  initial_twist_sigma_ang: 3.0
)###";

  mola::mapper_3d::Mapper3D nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  mrpt::poses::CPose3DPDFGaussian p0;
  p0.mean = mrpt::poses::CPose3D::Identity();
  p0.cov.setIdentity();
  p0.cov *= 0.01;
  nav.fuse_pose(mrpt::Clock::fromDouble(0.0), p0, "map");

  // A tight twist measurement of 2 m/s forward:
  mrpt::math::TTwist3D tw;
  tw.vx = 2.0;
  mrpt::math::CMatrixDouble66 twCov;
  twCov.setIdentity();
  twCov *= mrpt::square(0.05);
  nav.fuse_twist(mrpt::Clock::fromDouble(0.0), tw, twCov);

  const auto ret = nav.estimated_navstate(mrpt::Clock::fromDouble(0.0), "map");
  ASSERT_(ret.has_value());
  std::cout << "  twist vx=" << ret->twist.vx << "\n";
  ASSERT_NEAR_(ret->twist.vx, 2.0, 0.2);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_imu_leveling", test_imu_leveling},
    {"test_static_gnss_imu", test_static_gnss_imu},
    {"test_twist_fusion", test_twist_fusion},
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
