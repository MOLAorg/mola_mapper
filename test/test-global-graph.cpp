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
 * @brief  optimize_global_graph() on a synthetic drifting square loop.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GlobalGraph.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
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

  const auto noLoop = mola::mapper::optimize_global_graph(in, p);
  const double errNoLoop = position_error(noLoop.poses.back(), gt.back());

  // The front end revisits the start: last keyframe seen from the first one.
  mola::mapper::GlobalGraphLoopEdge lc;
  lc.from = 0;
  lc.to = gt.size() - 1;
  lc.relative_pose.mean = gt.back() - gt.front();
  lc.relative_pose.cov.setDiagonal(1e-4);
  in.loops.push_back(lc);

  const auto withLoop = mola::mapper::optimize_global_graph(in, p);
  const double errLoop = position_error(withLoop.poses.back(), gt.back());

  std::cout << "Loop closure: end position error " << errNoLoop << " m without, " << errLoop
            << " m with it\n";
  ASSERT_EQUAL_(withLoop.num_loop_factors, 1U);
  ASSERT_GT_(errNoLoop, 3.0);
  ASSERT_LT_(errLoop, 0.1);
}

void test_gnss_georeference_and_drift()
{
  const auto gt = ground_truth();
  GlobalGraphInput in;
  in.keyframes = drifting_odometry(gt, 0.1);

  // ENU <-> map: rotated 30 deg and shifted.
  const CPose3D T_enu_to_map(100.0, 50.0, 2.0, mrpt::DEG2RAD(30.0), 0, 0);
  const mrpt::math::TPoint3D antenna(0.1, 0.2, 0.9);

  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.3);
  for (std::size_t i = 0; i < gt.size(); i++) {
    const auto enu = (T_enu_to_map + gt[i]).composePoint(antenna);
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
    obs->sensorPose = CPose3D(antenna.x, antenna.y, antenna.z, 0, 0, 0);
    obs->setMsg(gga);
    obs->covariance_enu.emplace();
    obs->covariance_enu->setDiagonal(0.3 * 0.3);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);
    in.keyframes[i].aux_observations = sf;
  }
  in.geodetic_datum = DATUM;

  GlobalGraphParams p;
  p.use_imu_gravity = false;
  p.gnss_minimum_uncertainty = 0.3;

  const auto r = mola::mapper::optimize_global_graph(in, p);
  ASSERT_(r.georeferencing.has_value());
  ASSERT_EQUAL_(r.num_gnss_factors, gt.size());

  const auto & est = r.georeferencing->T_enu_to_map.mean;
  const double yawErrDeg =
    mrpt::RAD2DEG(std::abs(mrpt::math::wrapToPi(est.yaw() - T_enu_to_map.yaw())));
  double maxErr = 0;
  for (std::size_t i = 0; i < gt.size(); i++) {
    maxErr = std::max(maxErr, position_error(r.poses[i], gt[i]));
  }
  std::cout << "GNSS: T_enu_to_map yaw error " << yawErrDeg << " deg, translation error "
            << (est.translation() - T_enu_to_map.translation()).norm() << " m, max keyframe error "
            << maxErr << " m\n";
  ASSERT_LT_(yawErrDeg, 2.0);
  ASSERT_LT_((est.translation() - T_enu_to_map.translation()).norm(), 0.5);
  ASSERT_LT_(maxErr, 0.6);
}

}  // namespace

int main()
{
  try {
    test_loop_closure_removes_drift();
    test_gnss_georeference_and_drift();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
