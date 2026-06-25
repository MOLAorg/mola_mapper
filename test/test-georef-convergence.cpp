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
 * @file   test-georef-convergence.cpp
 * @brief  Geo-referencing convergence: synthetic odom+GNSS sweep across
 *         initial poses, kinematic models, and with/without GNSS.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Ported from mola_state_estimation_smoother's
 * test-navstate-odom-gnss-fusion.cpp (the "70-case" sweep), adapted to
 * Mapper3D's API (no sliding window; full ISAM2; needs
 * link_first_pose_to_reference_origin_sigma to fix the gauge for this
 * pure-odometry scenario).
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace mrpt::literals;

namespace
{
constexpr double GNSS_NOISE_XY_M = 0.05;
constexpr double GNSS_NOISE_Z_M = 0.05;
constexpr double MOTION_LIN_VX = 1.0;   // m/s
constexpr double MOTION_ANG_WZ = 0.5;   // rad/s
constexpr double ODOMETRY_NOISE_XY = 0.01;
constexpr double ODOMETRY_NOISE_PHI = 0.1_deg;

constexpr double MAXIMUM_SE3_FINAL_ERROR = 0.40;
constexpr double MAXIMUM_ENU2MAP_ROTATION_ERROR = 5.0_deg;

constexpr const char * ODOMETRY_NAME = "odom";

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr size_t numPoses = 80;
constexpr double T = 0.1;  // sensors period

using Pose = mrpt::poses::CPose3D;
using Kinematic = mola::mapper_3d::KinematicModel;

struct TestCase
{
  bool has_gnss = true;
  Pose pose;
  Kinematic model = Kinematic::ConstantVelocity;
};

const char * navStateParams =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.10
  estimate_geo_reference: true
  convergence_max_position_sigma: 1.0
  # Yaw is only observable indirectly (GNSS gives position only; heading
  # comes from the shape of the GNSS+odometry trajectory), so it converges
  # markedly slower than position; loosened vs the 5.0 deg smoother default
  # to stay robust over this test's 81-step / ~half-loop trajectory.
  convergence_max_orientation_sigma_deg: 8.0
  # Link the "map" frame origin to "odom" for this toy example (otherwise
  # "odom" would float around without any particular known displacement),
  # and REQUIRED while estimating geo-referencing: without it the whole
  # graph is gauge-free.
  link_first_pose_to_reference_origin_sigma: 1e-6
)###";

// Simulates a random trajectory on XY at a given latitude/longitude with an
// arbitrary initial heading, generates noisy local odometry + noisy GNSS
// measurements, and checks the geo-referenced trajectory (and convergence)
// is recovered.
void run_test(const TestCase & testCase)
{
  const auto actualInitialPoseWrtMap = testCase.pose;

  mola::mapper_3d::Mapper3D nav;
  if (VERBOSE) {
    nav.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
  }

  auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);
  cfgYaml["params"]["kinematic_model"] = mrpt::typemeta::enum2str(testCase.model);
  nav.initialize(cfgYaml);

  mrpt::poses::CPose3D actualVehiclePose = actualInitialPoseWrtMap;  // wrt "map" frame
  mrpt::poses::CPose2D odometryPose = mrpt::poses::CPose2D::Identity();  // wrt "odom" frame

  mrpt::topography::TGeodeticCoords actualVehicleInitialGeoCoords;
  actualVehicleInitialGeoCoords.lat = 4.0;
  actualVehicleInitialGeoCoords.lon = 3.0;
  actualVehicleInitialGeoCoords.height = 50.0;

  auto & rng = mrpt::random::getRandomGenerator();

  for (size_t i = 0; i <= numPoses; i++) {
    const double t = T * static_cast<double>(i);

    if (i > 0) {
      auto deltaPose2D = mrpt::poses::CPose2D(MOTION_LIN_VX * T, 0.0, MOTION_ANG_WZ * T);
      const auto deltaPose = mrpt::poses::CPose3D(deltaPose2D);
      actualVehiclePose = actualVehiclePose + deltaPose;

      deltaPose2D.x_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
      deltaPose2D.y_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
      deltaPose2D.phi_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_PHI));

      odometryPose = odometryPose + deltaPose2D;
    }

    mrpt::obs::CObservationOdometry obsOdo;
    obsOdo.timestamp = mrpt::Clock::fromDouble(t);
    obsOdo.sensorLabel = ODOMETRY_NAME;
    obsOdo.odometry = odometryPose;
    obsOdo.hasEncodersInfo = false;
    obsOdo.hasVelocities = false;
    nav.fuse_odometry(obsOdo, ODOMETRY_NAME);

    if (testCase.has_gnss) {
      mrpt::obs::CObservationGPS obsGps;
      obsGps.timestamp = mrpt::Clock::fromDouble(t);
      obsGps.sensorLabel = "gnss";

      mrpt::topography::TGeocentricCoords currentGeocentricCoords;
      mrpt::topography::ENUToGeocentric(
        {actualVehiclePose.x(), actualVehiclePose.y(), actualVehiclePose.z()},
        actualVehicleInitialGeoCoords, currentGeocentricCoords,
        mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

      mrpt::topography::TGeodeticCoords currentGeoCoords;
      mrpt::topography::geocentricToGeodetic(currentGeocentricCoords, currentGeoCoords);

      constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

      mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
      gga_msg.fields.latitude_degrees = currentGeoCoords.lat + rng.drawGaussian1D(0, gnss_noise_deg);
      gga_msg.fields.longitude_degrees =
        currentGeoCoords.lon + rng.drawGaussian1D(0, gnss_noise_deg);
      gga_msg.fields.altitude_meters =
        currentGeoCoords.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
      gga_msg.fields.fix_quality = 1;
      obsGps.setMsg(gga_msg);

      auto & cov = obsGps.covariance_enu.emplace();
      cov.setIdentity();
      cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
      cov(2, 2) = mrpt::square(GNSS_NOISE_Z_M);

      // Exercise the RawDataConsumer dispatch path too:
      nav.onNewObservation(std::make_shared<const mrpt::obs::CObservationGPS>(obsGps));
    }

    // Enforce updating the estimation from time to time (this is also what
    // flushes pending factors into iSAM2 and runs the convergence-gated
    // geo_reference publication, instead of leaving 81 steps' worth of
    // factors for one single batch update at the very end):
    if (i % 5 == 0) {
      const auto stateOpt =
        nav.estimated_navstate(mrpt::Clock::fromDouble(t), nav.parameters().reference_frame_name);
      if (stateOpt && VERBOSE) {
        std::cout << "  t=" << t << " " << stateOpt->pose.mean << "  GT=" << actualVehiclePose
                  << "\n";
      }
    }
  }

  const double last_t = T * static_cast<double>(numPoses);

  // Recover pose, in the reference frame:
  {
    const auto stateOpt =
      nav.estimated_navstate(mrpt::Clock::fromDouble(last_t), nav.parameters().reference_frame_name);
    ASSERT_(stateOpt.has_value());

    const auto estimatedPoseWrtMap = stateOpt->pose.mean;
    const double final_se3_error =
      mrpt::poses::Lie::SE<3>::log(estimatedPoseWrtMap - (actualVehiclePose - actualInitialPoseWrtMap))
        .norm();
    if (VERBOSE) {
      std::cout << "  (map frame) final_se3_error=" << final_se3_error << "\n";
    }
    ASSERT_LT_(final_se3_error, MAXIMUM_SE3_FINAL_ERROR);
  }

  // Recover pose, in the odometry frame:
  ASSERT_EQUAL_(nav.known_odometry_frame_ids().size(), 1U);
  {
    const auto stateOpt = nav.estimated_navstate(mrpt::Clock::fromDouble(last_t), ODOMETRY_NAME);
    ASSERT_(stateOpt.has_value());

    const double final_se3_error =
      mrpt::poses::Lie::SE<3>::log(stateOpt->pose.mean - (actualVehiclePose - actualInitialPoseWrtMap))
        .norm();
    if (VERBOSE) {
      std::cout << "  (odom frame) final_se3_error=" << final_se3_error << "\n";
    }
    ASSERT_LT_(final_se3_error, MAXIMUM_SE3_FINAL_ERROR);
  }

  // T_enu_to_map convergence:
  const auto T_enu_to_map = nav.estimated_T_enu_to_map();
  ASSERT_(T_enu_to_map.has_value());
  const double covDet = std::sqrt(T_enu_to_map->cov.det());
  if (VERBOSE) {
    std::cout << "  sqrt|cov(T_enu_to_map)|=" << covDet << "\n";
  }
  if (testCase.has_gnss) {
    // We should have converged:
    ASSERT_LT_(covDet, 1.0e-3);

    // And to a valid global orientation (translation is arbitrary):
    const double enu2map_rot_error =
      mrpt::poses::Lie::SO<3>::log((T_enu_to_map->mean - testCase.pose).getRotationMatrix()).norm();
    if (VERBOSE) {
      std::cout << "  log|error(T_enu_to_map)|=" << enu2map_rot_error << "\n";
    }
    ASSERT_LT_(enu2map_rot_error, MAXIMUM_ENU2MAP_ROTATION_ERROR);

    // Convergence-gated geo_reference publication + has_converged_localization():
    mrpt::poses::CPose3DPDFGaussian veh_pose_in_map;
    ASSERT_(nav.has_converged_localization(veh_pose_in_map));
    ASSERT_(nav.current_georeferencing().has_value());
  } else {
    // We have no clue about geo-referencing since we don't have GNSS:
    ASSERT_GT_(covDet, 1.0e3);

    mrpt::poses::CPose3DPDFGaussian veh_pose_in_map;
    ASSERT_(!nav.has_converged_localization(veh_pose_in_map));
  }

  for (const auto & odom_frame : nav.known_odometry_frame_ids()) {
    ASSERT_(nav.estimated_T_map_to_odometry_frame(odom_frame).has_value());
  }
}

void test_georef_convergence_sweep()
{
  auto & rng = mrpt::random::getRandomGenerator();

  constexpr int RANDOM_REPETITIONS = 5;

  const std::vector<TestCase> tests = {
    {false, Pose::Identity()},
    {false, Pose::FromTranslation(3.0, 2.0, 1.0)},
    {false, Pose::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg)},
    {true, Pose::Identity()},
    {true, Pose::FromTranslation(3.0, 2.0, 1.0)},
    {true, Pose::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg)},
  };

  int numErrors = 0;
  int numSuccess = 0;

  for (const auto kinModel : {Kinematic::ConstantVelocity, Kinematic::Tricycle}) {
    for (const auto & baseCase : tests) {
      TestCase tc = baseCase;
      tc.model = kinModel;

      for (int rep = 0; rep < RANDOM_REPETITIONS; rep++) {
        rng.randomize(1234 + rep);

        try {
          run_test(tc);
          numSuccess++;
        } catch (const std::exception & e) {
          numErrors++;
          std::cerr << "  [FAIL] pose=" << tc.pose << " kinematic="
                    << mrpt::typemeta::enum2str(kinModel) << " has_gnss=" << tc.has_gnss
                    << " rep=" << rep << "\n    " << mrpt::exception_to_str(e) << "\n";
        }
      }
    }
  }

  std::cout << "  " << numSuccess << " succeeded, " << numErrors << " failed (of "
            << (numSuccess + numErrors) << ")\n";
  ASSERT_EQUAL_(numErrors, 0);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_georef_convergence_sweep", test_georef_convergence_sweep},
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
