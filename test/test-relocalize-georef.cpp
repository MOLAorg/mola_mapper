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
 * @file   test-relocalize-georef.cpp
 * @brief  Relocalize mode: set_geo_reference() + convergence reporting.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Ported from mola_state_estimation_smoother's relocalize-mode regression
 * test. Covers the bug chain that mode exposed:
 *  - set_geo_reference() being a no-op leaves T_enu_to_map on its weak
 *    construction-time prior, so the absolute rotation measured by the
 *    IMU gravity/attitude factors is a gauge freedom the solver cannot
 *    resolve (it eventually throws inside iSAM2).
 *  - has_converged_localization() gated on `estimate_geo_reference`, which
 *    is false by definition here, so it could never report convergence.
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr const char * ODOMETRY_NAME = "odom_lidar";

// Relocalize mode: the geo-reference is FIXED (loaded with the map), never
// estimated. This is exactly the configuration in which the old convergence
// criterion was unconditionally false.
const char * relocalizeParams =
  R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  odometry_max_insert_rate_hz: 0.0
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 0.10
  # Dense synthetic keyframes hold ~1 accel sample each:
  imu_gravity_min_samples: 1
  imu_attitude_sigma_deg: 2.0
  # The synthetic IMU reports attitude already in the world/ENU frame, so
  # cancel the built-in "raw IMU yaw is North-referenced" +90 deg correction.
  imu_attitude_azimuth_offset_deg: -90.0
  estimate_geo_reference: false
  convergence_max_position_sigma: 1.0
  convergence_max_orientation_sigma_deg: 5.0
  link_first_pose_to_reference_origin_sigma: 1e-3
)###";

/// A known geo-reference, as a localization front end would supply after
/// loading a geo-referenced map.
mola::Georeferencing make_known_georef()
{
  mola::Georeferencing gr;
  gr.geo_coord.lat = 37.0;
  gr.geo_coord.lon = -2.0;
  gr.geo_coord.height = 100.0;

  // Pure translation: {enu} and {map} are rotationally aligned, so the
  // synthetic IMU attitude below (reported in {map}) is also the ENU attitude,
  // and the attitude factor does not fight the pinned map origin.
  gr.T_enu_to_map.mean =
    mrpt::poses::CPose3D::FromXYZYawPitchRoll(10.0, 20.0, 1.0, 0.0_deg, 0.0_deg, 0.0_deg);
  gr.T_enu_to_map.cov.setIdentity();
  gr.T_enu_to_map.cov *= mrpt::square(1e-3);
  return gr;
}

/// Feeds `numSteps` of LiDAR-odometry poses plus a flat-robot IMU, so the
/// gravity/attitude factors (the ones that need T_enu_to_map pinned) are
/// actually exercised.
void feed_trajectory(mola::mapper::Mapper & nav, size_t numSteps, double t0 = 0.0)
{
  auto & rng = mrpt::random::getRandomGenerator();
  constexpr double T = 0.05;

  mrpt::poses::CPose3D currentOdom = mrpt::poses::CPose3D::Identity();

  for (size_t i = 1; i <= numSteps; i++) {
    const auto time = mrpt::Clock::fromDouble(t0 + T * static_cast<double>(i));

    currentOdom = currentOdom + mrpt::poses::CPose3D(0.5 * T, 0, 0, 0.05 * T, 0, 0);

    // IMU first, and BETWEEN keyframe instants: a real IMU runs much faster
    // than the keyframe rate, and the per-keyframe interval window is open at
    // its lower end, so samples landing exactly on keyframe stamps would be
    // dropped and no attitude factor would ever be emitted.
    for (const double frac : {0.75, 0.25}) {
      const auto tImu = mrpt::Clock::fromDouble(t0 + T * (static_cast<double>(i) - frac));

      // Flat robot: gravity straight down in the vehicle frame.
      mrpt::obs::CObservationIMU obsImu;
      obsImu.timestamp = tImu;
      obsImu.sensorLabel = "imu";
      obsImu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0.0, 0.05));
      obsImu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0.0, 0.05));
      obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0.0, 0.05));

      // Absolute attitude, as a real relocalization setup provides it. Gravity
      // alone only levels roll/pitch, leaving absolute yaw unobservable; this
      // is what makes the orientation sigma converge at all.
      mrpt::math::CQuaternionDouble q;
      currentOdom.getAsQuaternion(q);
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_W, q.w());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
      obsImu.set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

      nav.fuse_imu(obsImu);
    }

    mrpt::poses::CPose3DPDFGaussian odomPdf;
    odomPdf.mean = currentOdom;
    odomPdf.cov.setIdentity();
    odomPdf.cov *= 1e-4;
    nav.fuse_pose(time, odomPdf, ODOMETRY_NAME);

    // Flush pending factors into iSAM2 regularly, so a gauge-freedom failure
    // surfaces here rather than in one big batch at the very end.
    if (i % 5 == 0) {
      const auto st = nav.estimated_navstate(time, nav.parameters().reference_frame_name);
      (void)st;
    }
  }
}

// ---------------------------------------------------------------------------
// set_geo_reference() before any fusion: the graph must stay solvable and the
// geo-reference must be adopted.
// ---------------------------------------------------------------------------
void test_set_geo_reference_before_fusion()
{
  mrpt::random::getRandomGenerator().randomize(1234);

  mola::mapper::Mapper nav;
  if (VERBOSE) {
    nav.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
  }
  nav.initialize(mrpt::containers::yaml::FromText(relocalizeParams));

  const auto georef = make_known_georef();
  nav.set_geo_reference(georef);

  // Adopted immediately (no GNSS fusion needed: it is a KNOWN value):
  const auto stored = nav.current_georeferencing();
  ASSERT_(stored.has_value());
  ASSERT_LT_((stored->T_enu_to_map.mean - georef.T_enu_to_map.mean).asVectorVal().norm(), 1e-6);

  // The whole point: with T_enu_to_map pinned this stays solvable. Before the
  // fix, the absolute rotation was a null space and iSAM2 threw partway in.
  feed_trajectory(nav, 100);

  ASSERT_GT_(nav.keyframe_count(), 10U);

  const auto st = nav.estimated_navstate(
    mrpt::Clock::fromDouble(0.05 * 100), nav.parameters().reference_frame_name);
  ASSERT_(st.has_value());

  // The geo-reference must not have drifted away from the value we pinned it to.
  const auto after = nav.current_georeferencing();
  ASSERT_(after.has_value());
  ASSERT_LT_((after->T_enu_to_map.mean - georef.T_enu_to_map.mean).asVectorVal().norm(), 1e-3);
}

// ---------------------------------------------------------------------------
// set_geo_reference() with an already-populated central map: the map is shared,
// persistent state and must survive the call.
// ---------------------------------------------------------------------------
void test_set_geo_reference_preserves_existing_map()
{
  mrpt::random::getRandomGenerator().randomize(1234);

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(relocalizeParams));

  feed_trajectory(nav, 50);
  const auto kfsBefore = nav.keyframe_count();
  ASSERT_GT_(kfsBefore, 10U);

  const auto georef = make_known_georef();
  nav.set_geo_reference(georef);

  // Keyframes preserved (NOT wiped like a full re-initialization would):
  ASSERT_EQUAL_(nav.keyframe_count(), kfsBefore);
  ASSERT_(nav.current_georeferencing().has_value());

  // ...and the graph keeps solving afterwards.
  feed_trajectory(nav, 50, /*t0=*/0.05 * 50);
  ASSERT_GT_(nav.keyframe_count(), kfsBefore);
}

// ---------------------------------------------------------------------------
// Convergence must actually be reportable in relocalize mode.
// ---------------------------------------------------------------------------
void test_relocalize_convergence_is_reported()
{
  mrpt::random::getRandomGenerator().randomize(1234);

  mola::mapper::Mapper nav;
  if (VERBOSE) {
    nav.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
  }
  nav.initialize(mrpt::containers::yaml::FromText(relocalizeParams));

  mrpt::poses::CPose3DPDFGaussian pose;

  // Nothing fused yet, and no geo-reference: not converged.
  ASSERT_(!nav.has_converged_localization(pose));

  nav.set_geo_reference(make_known_georef());
  feed_trajectory(nav, 100);

  // With a fixed geo-reference and a well-constrained trajectory, the vehicle
  // pose sigma clears the thresholds, so convergence IS reported. Before the
  // fix this returned false forever, since estimate_geo_reference is false.
  const bool converged = nav.has_converged_localization(pose);
  if (VERBOSE) {
    std::cout << "  converged=" << (converged ? "yes" : "no") << " pose=" << pose.mean << "\n";
  }
  ASSERT_(converged);
}

// ---------------------------------------------------------------------------
// Without any geo-reference at all (pure odometry), convergence stays false.
// ---------------------------------------------------------------------------
void test_no_georef_never_converges()
{
  mrpt::random::getRandomGenerator().randomize(1234);

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(relocalizeParams));

  feed_trajectory(nav, 100);

  mrpt::poses::CPose3DPDFGaussian pose;
  ASSERT_(!nav.has_converged_localization(pose));
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_no_georef_never_converges", test_no_georef_never_converges},
    {"test_relocalize_convergence_is_reported", test_relocalize_convergence_is_reported},
    {"test_set_geo_reference_before_fusion", test_set_geo_reference_before_fusion},
    {"test_set_geo_reference_preserves_existing_map",
     test_set_geo_reference_preserves_existing_map},
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
