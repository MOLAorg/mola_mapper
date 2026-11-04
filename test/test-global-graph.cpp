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
 * @file   test-global-graph.cpp
 * @brief  optimize_global_graph() on a synthetic drifting square loop, and the
 *         round trip of its saved input.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GlobalGraph.h>
#include <mola_mapper/GlobalGraphIO.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <vector>

namespace
{
using mola::mapper::GlobalGraphInput;
using mola::mapper::GlobalGraphKeyframe;
using mola::mapper::GlobalGraphParams;
using mrpt::poses::CPose3D;

const mrpt::topography::TGeodeticCoords DATUM(36.8287, -2.4073, 50.0);

/// Ground truth: a 20x20 m square, a keyframe each meter, ending back at the start.
std::vector<CPose3D> ground_truth()
{
  std::vector<CPose3D> gt;
  CPose3D p;
  for (int side = 0; side < 4; side++) {
    for (int i = 0; i < 20; i++) {
      gt.push_back(p);
      p = p + CPose3D(1.0, 0, 0, 0, 0, 0);
    }
    p = p + CPose3D(0, 0, 0, mrpt::DEG2RAD(90.0), 0, 0);
  }
  gt.push_back(p);
  return gt;
}

/// Odometry with a yaw drift per keyframe, in its own frame.
std::vector<GlobalGraphKeyframe> drifting_odometry(
  const std::vector<CPose3D> & gt, double yawDriftDeg)
{
  std::vector<GlobalGraphKeyframe> kfs;
  CPose3D odo = gt.front();
  for (std::size_t i = 0; i < gt.size(); i++) {
    if (i > 0) {
      const CPose3D step = gt[i] - gt[i - 1];
      odo = odo + step + CPose3D(0, 0, 0, mrpt::DEG2RAD(yawDriftDeg), 0, 0);
    }
    GlobalGraphKeyframe kf;
    kf.stamp = mrpt::Clock::fromDouble(1000.0 + static_cast<double>(i));
    kf.source_frame = "lidar_odom";
    kf.pose_in_source.mean = odo;
    kf.pose_in_source.cov.setDiagonal(1e-6);
    kf.initial_guess = odo;
    kfs.push_back(kf);
  }
  return kfs;
}

double position_error(const CPose3D & a, const CPose3D & b) { return (a - b).translation().norm(); }

void test_loop_closure_removes_drift()
{
  const auto gt = ground_truth();
  GlobalGraphInput in;
  in.keyframes = drifting_odometry(gt, 0.5);

  GlobalGraphParams p;
  p.use_gnss = false;
  p.use_imu_gravity = false;
  // An odometry noise model in line with this synthetic drift (0.5 deg per meter):
  p.odometry_noise_ang_deg_per_sqrt_m = 0.5;

  // Error of the loop end relative to its start (the drift the loop removes):
  constexpr std::size_t LOOP_START = 5;
  const auto loopError = [&](const std::vector<CPose3D> & est) {
    return position_error(est.back() - est[LOOP_START], gt.back() - gt[LOOP_START]);
  };

  const auto noLoop = mola::mapper::optimize_global_graph(in, p);
  const double errNoLoop = loopError(noLoop.poses);

  // The front end revisits the start: the last keyframe, seen from a keyframe 5 m
  // into the first side (not the identity, so the edge direction matters).
  mola::mapper::GlobalGraphLoopEdge lc;
  lc.from = LOOP_START;
  lc.to = gt.size() - 1;
  lc.relative_pose.mean = gt.back() - gt[lc.from];
  lc.relative_pose.cov.setDiagonal(1e-4);
  in.loops.push_back(lc);

  const auto withLoop = mola::mapper::optimize_global_graph(in, p);
  const double errLoop = loopError(withLoop.poses);

  // The same edge with from/to swapped is a different (wrong) constraint:
  auto inSwapped = in;
  std::swap(inSwapped.loops.back().from, inSwapped.loops.back().to);
  const auto swapped = mola::mapper::optimize_global_graph(inSwapped, p);
  const double errSwapped = loopError(swapped.poses);

  std::cout << "Loop closure: loop end position error " << errNoLoop << " m without, " << errLoop
            << " m with it, " << errSwapped << " m with it swapped\n";
  ASSERT_EQUAL_(withLoop.num_loop_factors, 1U);
  ASSERT_GT_(errNoLoop, 3.0);
  ASSERT_LT_(errLoop, 0.1);
  ASSERT_GT_(errSwapped, 1.0);
}

// ENU <-> map: rotated 30 deg and shifted.
const CPose3D T_ENU_TO_MAP(100.0, 50.0, 2.0, mrpt::DEG2RAD(30.0), 0, 0);
const mrpt::math::TPoint3D ANTENNA(0.1, 0.2, 0.9);

/// Attaches a GNSS fix (GGA, with that noise and reported sigma) to keyframes
/// [first, last] of `in`, from their ground truth poses.
void attach_gnss_fixes(
  GlobalGraphInput & in, const std::vector<CPose3D> & gt, std::size_t first, std::size_t last,
  double noiseSigma, double reportedSigma)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, noiseSigma);
  for (std::size_t i = first; i <= last; i++) {
    const auto enu = (T_ENU_TO_MAP + gt[i]).composePoint(ANTENNA);
    const mrpt::math::TPoint3D enuNoisy(enu.x + noise(rng), enu.y + noise(rng), enu.z + noise(rng));
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(enuNoisy, DATUM, geocentric, mrpt::topography::TEllipsoid());
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(geocentric, geo);

    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters = geo.height;
    gga.fields.fix_quality = 1;
    gga.fields.HDOP = 1.0;
    auto obs = mrpt::obs::CObservationGPS::Create();
    obs->timestamp = in.keyframes[i].stamp;
    obs->sensorPose = CPose3D(ANTENNA.x, ANTENNA.y, ANTENNA.z, 0, 0, 0);
    obs->setMsg(gga);
    obs->covariance_enu.emplace();
    obs->covariance_enu->setDiagonal(reportedSigma * reportedSigma);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);
    in.keyframes[i].aux_observations = sf;
  }
  in.geodetic_datum = DATUM;
}

void test_gnss_georeference_and_drift()
{
  const auto gt = ground_truth();
  GlobalGraphInput in;
  in.keyframes = drifting_odometry(gt, 0.1);
  attach_gnss_fixes(in, gt, 0, gt.size() - 1, 0.3, 0.3);

  GlobalGraphParams p;
  p.use_imu_gravity = false;
  p.gnss_minimum_uncertainty = 0.3;
  p.odometry_noise_ang_deg_per_sqrt_m = 0.2;  // a bit looser than the synthetic drift

  const auto r = mola::mapper::optimize_global_graph(in, p);
  ASSERT_(r.georeferencing.has_value());
  ASSERT_EQUAL_(r.num_gnss_factors, gt.size());

  const auto & est = r.georeferencing->T_enu_to_map.mean;
  const double yawErrDeg =
    mrpt::RAD2DEG(std::abs(mrpt::math::wrapToPi(est.yaw() - T_ENU_TO_MAP.yaw())));
  double maxErr = 0;
  for (std::size_t i = 0; i < gt.size(); i++) {
    maxErr = std::max(maxErr, position_error(r.poses[i], gt[i]));
  }
  std::cout << "GNSS: T_enu_to_map yaw error " << yawErrDeg << " deg, translation error "
            << (est.translation() - T_ENU_TO_MAP.translation()).norm() << " m, max keyframe error "
            << maxErr << " m\n";
  ASSERT_LT_(yawErrDeg, 2.0);
  ASSERT_LT_((est.translation() - T_ENU_TO_MAP.translation()).norm(), 0.5);
  ASSERT_LT_(maxErr, 0.8);  // the fixes have 0.3 m sigma
}

/** The geo-reference is only returned once observable: not from a single fix,
 *  nor from fixes much closer together than their sigmas (standing still), even
 *  if their reported sigmas are tiny (below the configured floor). */
void test_georeference_only_once_observable()
{
  const auto gt = ground_truth();
  GlobalGraphParams p;
  p.use_imu_gravity = false;

  {
    GlobalGraphInput in;
    in.keyframes = drifting_odometry(gt, 0.0);
    attach_gnss_fixes(in, gt, 0, 0, 0.01, 0.01);
    const auto r = mola::mapper::optimize_global_graph(in, p);
    ASSERT_EQUAL_(r.num_gnss_factors, 1U);
    ASSERT_(!r.georeferencing.has_value());
  }
  {
    // Standing still: the vehicle creeps 2 cm per keyframe, fixes with 1 cm
    // reported sigmas (floored to gnss_minimum_uncertainty = 0.5 m).
    std::vector<CPose3D> still;
    for (int i = 0; i < 5; i++) {
      still.emplace_back(0.02 * i, 0, 0, 0, 0, 0);
    }
    GlobalGraphInput in;
    in.keyframes = drifting_odometry(still, 0.0);
    attach_gnss_fixes(in, still, 0, still.size() - 1, 0.01, 0.01);
    const auto r = mola::mapper::optimize_global_graph(in, p);
    std::cout << "Geo-reference standing still: azimuth sigma "
              << mrpt::RAD2DEG(r.georef_sigma_yaw.value_or(-1)) << " deg\n";
    ASSERT_(!r.georeferencing.has_value());
  }
  {
    // Driving the first 20 m side: observable.
    GlobalGraphInput in;
    in.keyframes = drifting_odometry(gt, 0.0);
    attach_gnss_fixes(in, gt, 0, 19, 0.01, 0.01);
    const auto r = mola::mapper::optimize_global_graph(in, p);
    ASSERT_(r.georeferencing.has_value());
    const double yawErrDeg = mrpt::RAD2DEG(std::abs(
      mrpt::math::wrapToPi(r.georeferencing->T_enu_to_map.mean.yaw() - T_ENU_TO_MAP.yaw())));
    std::cout << "Geo-reference after a 20 m straight line: yaw error " << yawErrDeg
              << " deg, sigma " << mrpt::RAD2DEG(*r.georef_sigma_yaw) << " deg\n";
    ASSERT_LT_(yawErrDeg, 3.0);
  }
}

/** LiDAR odometry that loses 2 deg of heading in each 90 deg turn (as a short
 *  range LiDAR does in open areas), with a gyroscope that measures the turns
 *  (0.5 % scale error): the gyro factors keep the heading right. */
void test_gyro_fixes_turn_heading_errors()
{
  const auto gt = ground_truth();
  GlobalGraphInput in;
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 1.0);
  CPose3D odo = gt.front();
  for (std::size_t i = 0; i < gt.size(); i++) {
    GlobalGraphKeyframe kf;
    kf.stamp = mrpt::Clock::fromDouble(1000.0 + static_cast<double>(i));
    kf.source_frame = "lidar_odom";
    if (i > 0) {
      const CPose3D step = gt[i] - gt[i - 1];
      const bool turn = std::abs(step.yaw()) > mrpt::DEG2RAD(45.0);
      odo = odo + step + CPose3D(0, 0, 0, turn ? mrpt::DEG2RAD(2.0) : 0.0, 0, 0);
      // Gyro: the true rotation, with a scale error and noise, sigma as the mapper computes it.
      const double scaleErr = 0.005;
      const double dt = 1.0;          // [s] between keyframes
      const double biasSigma = 2e-4;  // [rad/s] from a bias source
      const double sigma = std::sqrt(
        mrpt::square(3e-4) * dt + mrpt::square(biasSigma * dt) +
        mrpt::square(scaleErr * std::abs(step.yaw())));
      kf.gyro_rotation_from_prev =
        CPose3D(0, 0, 0, step.yaw() * (1.0 + scaleErr) + sigma * 0.5 * noise(rng), 0, 0);
      kf.gyro_rotation_sigma = sigma;
    }
    kf.pose_in_source.mean = odo;
    kf.pose_in_source.cov.setDiagonal(1e-6);
    kf.initial_guess = odo;
    in.keyframes.push_back(kf);
  }

  GlobalGraphParams p;
  p.use_gnss = false;
  p.use_imu_gravity = false;

  p.use_gyro = false;
  const auto noGyro = mola::mapper::optimize_global_graph(in, p);
  p.use_gyro = true;
  const auto withGyro = mola::mapper::optimize_global_graph(in, p);

  const auto yawErr = [&](const CPose3D & e) {
    return std::abs(mrpt::math::wrapToPi(e.yaw() - gt.back().yaw()));
  };
  std::cout << "Gyro: final heading error " << mrpt::RAD2DEG(yawErr(noGyro.poses.back()))
            << " deg without, " << mrpt::RAD2DEG(yawErr(withGyro.poses.back()))
            << " deg with gyro factors; end position error "
            << position_error(noGyro.poses.back(), gt.back()) << " vs "
            << position_error(withGyro.poses.back(), gt.back()) << " m\n";
  ASSERT_EQUAL_(noGyro.num_gyro_factors, 0U);
  ASSERT_EQUAL_(withGyro.num_gyro_factors, gt.size() - 1);
  ASSERT_GT_(yawErr(noGyro.poses.back()), mrpt::DEG2RAD(7.0));
  ASSERT_LT_(yawErr(withGyro.poses.back()), mrpt::DEG2RAD(2.5));
  ASSERT_LT_(
    position_error(withGyro.poses.back(), gt.back()),
    0.5 * position_error(noGyro.poses.back(), gt.back()));
}

/** The saved graph input reloads with everything the solver uses, including
 *  the geo-reference guess, so an offline re-optimization starts where the
 *  online one did. */
void test_graph_input_round_trip()
{
  const auto gt = ground_truth();
  GlobalGraphInput in;
  in.keyframes = drifting_odometry(gt, 0.1);
  attach_gnss_fixes(in, gt, 0, 9, 0.3, 0.3);
  in.keyframes[3].gyro_rotation_from_prev = CPose3D(0, 0, 0, 0.1, 0.02, -0.01);
  in.keyframes[3].gyro_rotation_sigma = 0.002;
  mola::mapper::GlobalGraphLoopEdge lc;
  lc.from = 0;
  lc.to = gt.size() - 1;
  lc.relative_pose.mean = CPose3D(0.1, -0.2, 0, 0.05, 0, 0);
  lc.relative_pose.cov.setDiagonal(1e-4);
  in.loops.push_back(lc);
  const CPose3D guess(
    100.0, 50.0, 2.0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(2.0), mrpt::DEG2RAD(-1.0));
  in.T_enu_to_map_guess = guess;

  const auto prefix =
    (std::filesystem::temp_directory_path() / "mola_mapper_test_graph_io").string();
  mola::mapper::save_global_graph_input(in, prefix);
  const auto out = mola::mapper::load_global_graph_input(prefix);

  ASSERT_EQUAL_(out.keyframes.size(), in.keyframes.size());
  for (std::size_t i = 0; i < in.keyframes.size(); i++) {
    const auto & a = in.keyframes[i];
    const auto & b = out.keyframes[i];
    ASSERT_EQUAL_(a.source_frame, b.source_frame);
    ASSERT_LT_(std::abs(mrpt::Clock::toDouble(a.stamp) - mrpt::Clock::toDouble(b.stamp)), 1e-5);
    ASSERT_LT_(position_error(a.pose_in_source.mean, b.pose_in_source.mean), 1e-9);
    ASSERT_LT_(position_error(a.initial_guess, b.initial_guess), 1e-9);
    ASSERT_EQUAL_(a.gyro_rotation_from_prev.has_value(), b.gyro_rotation_from_prev.has_value());
    // Keyframes without observations reload with an empty set:
    ASSERT_EQUAL_(a.aux_observations ? a.aux_observations->size() : 0U, b.aux_observations->size());
  }
  ASSERT_LT_(std::abs(out.keyframes[3].gyro_rotation_from_prev->yaw() - 0.1), 1e-9);
  ASSERT_EQUAL_(out.loops.size(), 1U);
  ASSERT_EQUAL_(out.loops[0].from, lc.from);
  ASSERT_EQUAL_(out.loops[0].to, lc.to);
  ASSERT_LT_(
    (out.loops[0].relative_pose.mean.asVectorVal() - lc.relative_pose.mean.asVectorVal()).norm(),
    1e-9);
  ASSERT_LT_((out.loops[0].relative_pose.cov - lc.relative_pose.cov).norm(), 1e-12);
  ASSERT_(out.geodetic_datum.has_value());
  ASSERT_LT_(std::abs(out.geodetic_datum->lat.decimal_value - DATUM.lat.decimal_value), 1e-9);
  ASSERT_(out.T_enu_to_map_guess.has_value());
  ASSERT_LT_((out.T_enu_to_map_guess->asVectorVal() - guess.asVectorVal()).norm(), 1e-9);

  // Saved again without the optional values, with the same prefix: they do
  // not come back from the earlier files.
  in.T_enu_to_map_guess.reset();
  in.geodetic_datum.reset();
  mola::mapper::save_global_graph_input(in, prefix);
  const auto out2 = mola::mapper::load_global_graph_input(prefix);
  ASSERT_(!out2.T_enu_to_map_guess.has_value());
  ASSERT_(!out2.geodetic_datum.has_value());

  // A file that cannot be written is an error, not a silent no-op:
  bool threw = false;
  try {
    mola::mapper::save_tum_trajectory({}, "/nonexistent-dir/x.tum");
  } catch (const std::exception &) {
    threw = true;
  }
  ASSERT_(threw);

  for (const char * ext :
       {".keyframes.txt", ".loops.txt", ".datum.txt", ".enu_to_map_guess.txt", ".aux.simplemap"}) {
    std::filesystem::remove(prefix + ext);
  }
}

}  // namespace

int main()
{
  try {
    test_loop_closure_removes_drift();
    test_gyro_fixes_turn_heading_errors();
    test_gnss_georeference_and_drift();
    test_georeference_only_once_observable();
    test_graph_input_round_trip();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
