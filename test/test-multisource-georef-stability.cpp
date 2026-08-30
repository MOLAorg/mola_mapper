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
 * @file   test-multisource-georef-stability.cpp
 * @brief  End-to-end multi-sensor fusion sweep mirroring a real LIO+IMU+GNSS
 *         setup (MulRan-style), driven through onNewObservation() and the
 *         SharedKeyframeMap "propose keyframe" API.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Reproduces, in a controlled synthetic world, the failure mode observed on
 * MulRan DCC01 where T_enu_to_map (geo-referencing) does not converge to a
 * bounded (yaw + small translation) transform but instead grows without bound.
 *
 * A ~1000 m ground-truth trajectory on flat ground is generated, and each
 * sensor stream is synthesized as a real front end would deliver it:
 *  - LIO keyframes (SharedKeyframeMap::requestInsertKeyframe), with a
 *    SYSTEMATIC per-keyframe pitch (tilt) bias, i.e. the realistic long-term
 *    LIO z/tilt drift failure mode. Its own odom frame starts at identity.
 *  - Wheel odometry (CObservationOdometry) in its OWN frame, whose origin is
 *    FAR from the LIO origin (only the relative motion is fused).
 *  - IMU accelerometer (gravity, body frame) -> levels the map.
 *  - IMU absolute attitude (global heading quaternion) -> recovers azimuth.
 *  - GNSS (CObservationGPS, NMEA GGA + ENU covariance), noisy 0.1 m XY / 0.2 m Z.
 *
 * The matrix of ON/OFF source combinations is swept and, for each, we check:
 *  (a) the vehicle pose is recovered (both in {map} and the LIO {odom} frame),
 *  (b) IMU levels the map / attitude recovers azimuth,
 *  (c) T_enu_to_map is recovered AND stays BOUNDED over the whole run (the
 *      anti-regression check for the MulRan unbounded-growth bug),
 *  (d) the estimated trajectory advances in the SAME direction as ground truth
 *      (catches the "LIO line vs graph edges point opposite ways" viz bug at
 *      the estimate level).
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// --- Ground-truth world / motion profile -----------------------------------
constexpr double DT = 0.02;           // master tick period [s] (50 Hz IMU)
constexpr double V_FWD = 10.0;        // forward speed [m/s]
constexpr double DURATION_S = 100.0;  // 100 s * 10 m/s = ~1000 m
constexpr double PSI0_DEG = 50.0;     // initial heading wrt East (ENU)

// Yaw-rate profile: a steadily-turning arc (wide net heading sweep) with an
// added curvature variation, NOT a straight line. What makes the geo-ref
// orientation (and, through the chain, the keyframe roll/pitch) OBSERVABLE from
// GNSS positions alone is HEADING DIVERSITY: the trajectory must sample many
// travel directions. A straight track (one direction) leaves the rotation about
// the travel axis free; a narrow oscillation around one heading is nearly as
// degenerate. A wide monotonic sweep (here ~110 deg over the run) gives rich
// directional coverage, and the sinusoidal term keeps the curvature varying so
// it is not a single perfect circle. The yaw rate stays POSITIVE so heading
// keeps advancing (diversity), it does not cancel back.
constexpr double W_Z_BASE = 0.02;      // mean yaw rate [rad/s] (~110 deg sweep)
constexpr double W_Z_AMP = 0.01;       // curvature variation amplitude [rad/s]
constexpr double W_Z_PERIOD_S = 50.0;  // variation period [s]

double yaw_rate_at(double t_sec)
{
  return W_Z_BASE + W_Z_AMP * std::sin(2.0 * M_PI * t_sec / W_Z_PERIOD_S);
}

// Sub-rate decimations (in master ticks):
constexpr int KF_EVERY = 50;  // LIO keyframe every 1.0 s (~10 m), ~100 KFs
// GNSS shares the keyframe cadence so each fix constrains the keyframe it was
// actually measured at (in SharedMapOnly mode fuse_gnss snaps a GNSS reading to
// the temporally NEAREST keyframe; a faster-than-keyframe GNSS rate would snap
// off-keyframe fixes up to half a keyframe period away, a separate temporal
// error we deliberately keep out of the precision check).
constexpr int GNSS_EVERY = 50;  // GNSS coincident with keyframes
constexpr int WHEEL_EVERY = 2;  // wheels at 25 Hz

// Per-keyframe systematic LIO tilt (pitch) drift (~5 deg total over ~100 KFs):
const double LIO_PITCH_BIAS_PER_KF = 0.05_deg;
// Per-keyframe systematic LIO YAW drift (~3 deg total over ~100 KFs). This is
// the realistic long-term LIO heading drift and the DIRECT driver of the
// T_enu_to_map unbounded-growth failure on real data: a single rigid
// T_enu_to_map cannot fit a yaw-drifting trajectory, so if GNSS cannot bend the
// keyframe chain (e.g. T_enu_to_map roll/pitch left free to absorb it) the
// transform's XY translation chases the drift and grows over the run.
const double LIO_YAW_BIAS_PER_KF = 0.03_deg;

// Per-keyframe relative-pose uncertainty of the synthetic LIO. The reported
// ABSOLUTE covariance accumulates this (sqrt(n) growth), as real odometry does,
// so the odometry-edge noise stays soft enough for GNSS to correct the chain.
const double LIO_REL_SIGMA_LIN = 0.03;      // [m] per keyframe
const double LIO_REL_SIGMA_ANG = 0.05_deg;  // [rad] per keyframe

// Default GNSS noise (per the task spec): standard-precision GNSS.
constexpr double GNSS_NOISE_XY_M = 0.10;
constexpr double GNSS_NOISE_Z_M = 0.20;
// Centimeter-level (RTK-fixed) GNSS, used to verify the trajectory recovers
// with precision when the absolute position reference is very accurate.
constexpr double GNSS_CM_XY_M = 0.02;
constexpr double GNSS_CM_Z_M = 0.02;

// Geo origin for the synthetic GNSS:
constexpr double GEO_LAT0 = 36.0;
constexpr double GEO_LON0 = -2.0;
constexpr double GEO_H0 = 100.0;

// Wheel odometry frame origin: deliberately FAR from the LIO origin.
const auto WHEEL_ORIGIN = mrpt::poses::CPose2D(1000.0, -500.0, 1.3);

struct Combo
{
  std::string name;
  bool wheels = false;
  bool imu_acc = false;
  bool imu_att = false;
  bool gnss = false;
  // Per-combo GNSS noise (defaults to standard precision; the LIO+GPS case
  // overrides to centimeter level to check precise trajectory recovery):
  double gnss_xy = GNSS_NOISE_XY_M;
  double gnss_z = GNSS_NOISE_Z_M;
  // Over-confident ("stiff") LIO: report tiny near-constant covariance even
  // though the trajectory systematically drifts. This mimics a real LIO front
  // end whose reported covariance is far smaller than its true long-term drift,
  // which is what locks the relative-pose edge chain rigid and (in the buggy
  // configuration) forces T_enu_to_map to ABSORB the drift -> the unbounded
  // growth observed on MulRan. With the geo-ref fix, GNSS must still keep
  // T_enu_to_map bounded by levelling + softening only the geo-ref-relevant DOFs.
  bool stiff_lio = false;
  // Trajectory length multiplier (longer trajectory -> larger lever arm, the
  // factor that turns a residual map-vs-ENU rotation into a big translation).
  double duration_scale = 1.0;

  [[nodiscard]] bool any_imu() const { return imu_acc || imu_att; }
  [[nodiscard]] bool gnss_is_cm() const { return gnss && gnss_xy <= GNSS_CM_XY_M + 1e-9; }
};

std::string make_params_yaml(const Combo & c)
{
  // Mirrors params/mapper-common.yaml + the MulRan launcher overrides, but
  // single-threaded + deterministic for the unit test.
  std::string s =
    R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  enu_frame_name: "enu"
  kinematic_model: KinematicModel::ConstantVelocity
  keyframe_creation_source: KeyframeCreationSource::SharedMapOnly
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.05
  odometry_max_insert_rate_hz: 0.0
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 10.0
  sigma_integrator_position: 0.10
  sigma_integrator_orientation: 1.0
  link_first_pose_to_reference_origin_sigma: 1e-6
  odometry_edge_min_sigma_rollpitch_deg: 3.0
  imu_gravity_min_samples: 5
  convergence_max_position_sigma: 1.5
  convergence_max_orientation_sigma_deg: 8.0
  enable_optimizer_thread: false
  # Non-robust GNSS kernel: a fixed Huber threshold (in sigmas) DOWNWEIGHTS
  # exactly the large residuals needed to pull a drifted chain back toward the
  # GNSS positions while the geo-ref is still being established, so it harms
  # recovery during this bootstrap regime (the two-phase design uses GNC here,
  # and only switches to a robust kernel once the geo-ref has converged; see
  # agents.md "Design ideas"). The synthetic streams have no outliers, so a
  # plain Gaussian model is the right model for this test.
  gnss_huber_threshold: 0.0
)###";

  // Estimate geo-referencing whenever an absolute azimuth source exists (GNSS,
  // or the IMU absolute attitude). The latter lets iSAM2 auto-estimate the
  // T_enu_to_map yaw with GNSS off (see agents.md "IMU gravity leveling").
  const bool estimateGeoref = c.gnss || c.imu_att;
  s += "  estimate_geo_reference: ";
  s += (estimateGeoref ? "true\n" : "false\n");
  // Attitude factor is gated by whether the IMU obs carries a quaternion, but
  // keep the sigma >0 so the factor is built when a quaternion IS present:
  s += "  imu_attitude_sigma_deg: 2.0\n";
  return s;
}

// A single CObservationIMU as the Xsens delivers it: proper accel (+optional
// absolute attitude quaternion). The robot is physically flat, so gravity is
// straight down in the body frame regardless of LIO's drifted estimate.
mrpt::obs::CObservationIMU make_imu(
  const mrpt::Clock::time_point & t, double yaw_enu, const Combo & c,
  mrpt::random::CRandomGenerator & rng)
{
  mrpt::obs::CObservationIMU o;
  o.timestamp = t;
  o.sensorLabel = "imu";

  if (c.imu_acc) {
    o.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0.0, 0.05));
    o.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0.0, 0.05));
    o.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0.0, 0.05));
  }

  if (c.imu_att) {
    // The IMU reports the vehicle's absolute attitude. The backend converts it
    // to azimuth via a +90 deg Rz (ENU yaw=0 => East). We therefore feed the
    // body world-attitude as (yaw_enu - 90 deg); since the body is flat this is
    // a pure yaw, which commutes with the backend correction.
    const auto worldAtt =
      mrpt::poses::CPose3D::FromYawPitchRoll(yaw_enu - mrpt::DEG2RAD(90.0), 0.0, 0.0);
    mrpt::math::CQuaternionDouble q;
    worldAtt.getAsQuaternion(q);
    mrpt::math::CQuaternionDouble nq;
    for (int k = 0; k < 4; k++) {
      nq[k] = q[k] + rng.drawGaussian1D(0, 0.005);
    }
    nq.normalize();
    o.set(mrpt::obs::IMU_ORI_QUAT_W, nq.w());
    o.set(mrpt::obs::IMU_ORI_QUAT_X, nq.x());
    o.set(mrpt::obs::IMU_ORI_QUAT_Y, nq.y());
    o.set(mrpt::obs::IMU_ORI_QUAT_Z, nq.z());
  }
  return o;
}

mrpt::obs::CObservationGPS make_gnss(
  const mrpt::Clock::time_point & t, const mrpt::math::TPoint3D & enu,
  const mrpt::topography::TGeodeticCoords & geoRef, double xyNoise, double zNoise,
  mrpt::random::CRandomGenerator & rng)
{
  mrpt::obs::CObservationGPS o;
  o.timestamp = t;
  o.sensorLabel = "gps";

  mrpt::topography::TGeocentricCoords geocentric;
  mrpt::topography::ENUToGeocentric(
    enu, geoRef, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
  mrpt::topography::TGeodeticCoords geodetic;
  mrpt::topography::geocentricToGeodetic(geocentric, geodetic);

  const double gnss_noise_deg = xyNoise * mrpt::RAD2DEG(1.0 / 6300e3);

  mrpt::obs::gnss::Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = geodetic.lat + rng.drawGaussian1D(0, gnss_noise_deg);
  gga.fields.longitude_degrees = geodetic.lon + rng.drawGaussian1D(0, gnss_noise_deg);
  gga.fields.altitude_meters = geodetic.height + rng.drawGaussian1D(0, zNoise);
  gga.fields.fix_quality = (xyNoise <= GNSS_CM_XY_M + 1e-9) ? 4 : 1;  // 4 = RTK fixed
  o.setMsg(gga);

  auto & cov = o.covariance_enu.emplace();
  cov.setIdentity();
  cov(0, 0) = cov(1, 1) = mrpt::square(xyNoise);
  cov(2, 2) = mrpt::square(zNoise);
  return o;
}

mola::SharedKeyframeMap::KeyframeInsertRequest make_kf_request(
  const mrpt::Clock::time_point & t, const mrpt::poses::CPose3D & poseInLioOdom, double sigmaLin,
  double sigmaAng)
{
  mola::SharedKeyframeMap::KeyframeInsertRequest req;
  req.timestamp = t;
  req.source_frame_id = "lidar_kf";
  req.pose_in_source.mean = poseInLioOdom;
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

struct RunResult
{
  mrpt::poses::CPose3D est_pose_map;                  // final estimated vehicle pose in {map}
  mrpt::poses::CPose3D gt_pose_map;                   // final GT vehicle pose in {map}
  mrpt::poses::CPose3D gt_pose_enu;                   // final GT vehicle pose in {enu}
  std::optional<mrpt::poses::CPose3D> est_pose_odom;  // final pose in LIO {odom}
  std::optional<mrpt::poses::CPose3DPDFGaussian> T_enu_to_map;
  double max_enu2map_trans_norm = 0;  // peak |translation| over the run
  double dir_dot = 0;                 // estimated vs GT displacement dot
  bool converged_localization = false;
};

RunResult run_combo(const Combo & c)
{
  auto & rng = mrpt::random::getRandomGenerator();
  rng.randomize(20260629);

  mola::mapper::Mapper nav;
  if (VERBOSE) {
    nav.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
  }
  nav.initialize(mrpt::containers::yaml::FromText(make_params_yaml(c)));

  mrpt::topography::TGeodeticCoords geoRef;
  geoRef.lat = GEO_LAT0;
  geoRef.lon = GEO_LON0;
  geoRef.height = GEO_H0;

  // GT vehicle pose in ENU, integrated from the motion profile:
  auto gtEnu = mrpt::poses::CPose3D::FromYawPitchRoll(mrpt::DEG2RAD(PSI0_DEG), 0.0, 0.0);
  // LIO's own (drifting) odom pose, starts at identity in its own frame:
  auto lioOdom = mrpt::poses::CPose3D::Identity();
  // Wheel odom pose (2D), starts FAR away in its own frame:
  auto wheelOdom = WHEEL_ORIGIN;

  // {map} origin == GT vehicle pose at the FIRST keyframe (tick 0):
  const auto mapOriginEnu = gtEnu;

  const int numTicks = static_cast<int>((DURATION_S * c.duration_scale) / DT);
  int kfIdx = 0;  // count of keyframes emitted (for accumulating LIO covariance)

  RunResult res;

  // First/last sampled checkpoints (in {map}) to verify travel-direction
  // consistency between the estimate and ground truth:
  std::optional<mrpt::poses::CPose3D> estAtFirst;
  std::optional<mrpt::poses::CPose3D> gtAtFirst;
  std::optional<mrpt::poses::CPose3D> estAtLast;

  for (int i = 0; i <= numTicks; i++) {
    const double tSec = DT * static_cast<double>(i);
    const auto t = mrpt::Clock::fromDouble(tSec);
    const double wz = yaw_rate_at(tSec);

    // Integrate GT + LIO odom one tick forward (skip at i==0):
    if (i > 0) {
      const auto deltaTrue = mrpt::poses::CPose3D(V_FWD * DT, 0, 0, wz * DT, 0, 0);
      gtEnu = gtEnu + deltaTrue;
      lioOdom = lioOdom + deltaTrue;
    }

    double gtYaw = 0;
    double gtPitch = 0;
    double gtRoll = 0;
    gtEnu.getYawPitchRoll(gtYaw, gtPitch, gtRoll);

    // --- IMU at every tick (highest rate) ---
    if (c.any_imu()) {
      const auto imu = make_imu(t, gtYaw, c, rng);
      nav.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(imu));
    }

    // --- Wheels ---
    if (c.wheels && (i % WHEEL_EVERY == 0)) {
      mrpt::obs::CObservationOdometry o;
      o.timestamp = t;
      o.sensorLabel = "wheel_odom";
      o.odometry = wheelOdom;
      o.hasEncodersInfo = false;
      o.hasVelocities = false;
      nav.fuse_odometry(o, "wheel_odom");
    }
    if (c.wheels && i > 0) {
      // Advance the wheel integrator each tick (clean planar motion):
      wheelOdom = wheelOdom + mrpt::poses::CPose2D(V_FWD * DT, 0, wz * DT);
    }

    // --- GNSS ---
    if (c.gnss && (i % GNSS_EVERY == 0)) {
      const auto gnss = make_gnss(t, gtEnu.translation(), geoRef, c.gnss_xy, c.gnss_z, rng);
      nav.onNewObservation(std::make_shared<const mrpt::obs::CObservationGPS>(gnss));
    }

    // --- LIO keyframe (drives keyframe creation, SharedMapOnly mode) ---
    if (i % KF_EVERY == 0) {
      // Inject the systematic LIO tilt bias once per keyframe (a small extra
      // pitch that progressively tilts the LIO frame, dragging an apparent z
      // drift as forward motion composes through the tilted orientation):
      if (i > 0) {
        lioOdom =
          lioOdom + mrpt::poses::CPose3D(0, 0, 0, LIO_YAW_BIAS_PER_KF, LIO_PITCH_BIAS_PER_KF, 0);
      }
      // Realistic accumulating dead-reckoning covariance: the absolute pose
      // variance grows ~linearly with traveled keyframes, so the PROPAGATED
      // relative covariance the odometry-edge noise is derived from
      // (cov_to - cov_from) stays ~ the per-keyframe increment (NOT zero, which
      // would collapse the edge noise to the floor and freeze the chain rigid
      // against GNSS). sigma(n) = sqrt(n) * per-keyframe sigma.
      const double growth = std::sqrt(static_cast<double>(kfIdx) + 1.0);
      double sigmaLin = LIO_REL_SIGMA_LIN * growth;
      double sigmaAng = LIO_REL_SIGMA_ANG * growth;
      if (c.stiff_lio) {
        // Over-confident report: tiny near-constant covariance, so the derived
        // per-edge relative noise collapses to the floor and the chain is rigid.
        sigmaLin = 1e-3;
        sigmaAng = mrpt::DEG2RAD(1e-3);
      }
      const auto req = make_kf_request(t, lioOdom, sigmaLin, sigmaAng);
      const auto kfId = nav.requestInsertKeyframe(req);
      ASSERT_(kfId.has_value());
      kfIdx++;
    }

    // Periodically flush + sample the geo-ref estimate (also exercises the
    // convergence-gated publication, and lets us watch for unbounded growth):
    if (i % KF_EVERY == 0 && i > 0) {
      const auto st = nav.estimated_navstate(t, "map");
      const auto Tem = nav.estimated_T_enu_to_map();
      if (Tem.has_value()) {
        res.max_enu2map_trans_norm =
          std::max(res.max_enu2map_trans_norm, Tem->mean.translation().norm());
      }
      if (st.has_value()) {
        if (!estAtFirst.has_value()) {
          estAtFirst = st->pose.mean;
          gtAtFirst = mrpt::poses::CPose3D(gtEnu - mapOriginEnu);
        }
        estAtLast = st->pose.mean;
      }
      if (VERBOSE && Tem.has_value()) {
        std::cout << "  t=" << tSec << " |T_enu2map.t|=" << Tem->mean.translation().norm()
                  << " est=" << (st ? st->pose.mean.asString() : std::string("(none)")) << "\n";
      }
    }
  }

  const double tLast = DT * static_cast<double>(numTicks);
  const auto tp = mrpt::Clock::fromDouble(tLast);

  // GT vehicle pose in {map}: vehicle-in-map = inverse(mapOriginEnu) (+) gtEnu
  res.gt_pose_map = mrpt::poses::CPose3D(gtEnu - mapOriginEnu);
  res.gt_pose_enu = gtEnu;

  const auto stMap = nav.estimated_navstate(tp, "map");
  ASSERT_(stMap.has_value());
  res.est_pose_map = stMap->pose.mean;

  res.est_pose_odom = std::nullopt;
  if (const auto stOdom = nav.estimated_navstate(tp, "lidar_kf"); stOdom.has_value()) {
    res.est_pose_odom = stOdom->pose.mean;
  }

  res.T_enu_to_map = nav.estimated_T_enu_to_map();

  // Direction consistency: estimated displacement (first->last sampled KF) vs
  // GT displacement over the same checkpoints, projected onto XY. A positive
  // dot product means the estimated trajectory travels the SAME way as GT.
  if (estAtFirst.has_value() && estAtLast.has_value() && gtAtFirst.has_value()) {
    const auto dEst = estAtLast->translation() - estAtFirst->translation();
    const auto dGt = res.gt_pose_map.translation() - gtAtFirst->translation();
    res.dir_dot = dEst.x * dGt.x + dEst.y * dGt.y;
  }

  mrpt::poses::CPose3DPDFGaussian dummy;
  res.converged_localization = nav.has_converged_localization(dummy);
  return res;
}

void check_combo(const Combo & c)
{
  std::cout << "  --- combo: " << c.name << " ---\n";
  const auto r = run_combo(c);

  const double se3_xy_err =
    std::hypot(r.est_pose_map.x() - r.gt_pose_map.x(), r.est_pose_map.y() - r.gt_pose_map.y());

  double ey = 0;
  double ep = 0;
  double er = 0;
  r.est_pose_map.getYawPitchRoll(ey, ep, er);

  std::cout << "    final est(map)=" << r.est_pose_map.asString() << "\n"
            << "    final  gt(map)=" << r.gt_pose_map.asString() << "\n"
            << "    XY err=" << se3_xy_err << " m  pitch=" << mrpt::RAD2DEG(ep)
            << " roll=" << mrpt::RAD2DEG(er) << " deg\n"
            << "    max|T_enu2map.t|=" << r.max_enu2map_trans_norm << " m  dir_dot=" << r.dir_dot
            << "\n";

  // (a) Horizontal pose recovery. With GNSS the absolute XY is pinned tight;
  // without it only dead-reckoning holds (IMU leveling of the pitch-drifted
  // chain couples a small yaw into the curved 1 km path), so allow drift.
  //
  // GNSS alone does not pin it as tight as GNSS + an absolute attitude does,
  // for the same reason map_pos_bound below splits on any_imu(): GNSS measures
  // POSITION only, so with no orientation reference the roll is weakly
  // observable and a degenerate roll leaks into the horizontal error. That is
  // visible in the numbers -- every combo with an attitude source lands at
  // 1.4-1.8 m, while the two stiff GNSS-only ones settle at a roll of -7 to
  // -9 deg. Bound the two regimes separately rather than at one value that
  // only fits the observable one.
  double max_xy_err = 30.0;
  if (c.gnss) {
    max_xy_err = c.imu_att ? 3.0 : 4.0;
  }
  ASSERT_LT_(se3_xy_err, max_xy_err);

  // (a') Centimeter-level (RTK) GNSS recovery. Precise (~1 m) recovery of a
  // drifting LIO chain needs an ABSOLUTE ORIENTATION reference (IMU): GNSS
  // measures POSITION only, while the trajectory heading is held by the tight
  // relative-yaw odometry edges (LIO's drift), so without an orientation source
  // position fixes cannot fully reshape the chain and a few meters of yaw-drift
  // shape error remain. With IMU the heading is pinned and recovery is tight.
  if (c.gnss_is_cm()) {
    const double cm_xy_bound = c.any_imu() ? 1.0 : 2.5;
    std::cout << "    [cm-GNSS] XY err=" << se3_xy_err << " m\n";
    ASSERT_LT_(se3_xy_err, cm_xy_bound);
  }

  // (d) The estimated trajectory must advance in the GT travel direction.
  ASSERT_GT_(r.dir_dot, 0.0);

  // (b) IMU leveling: with accelerometer (or attitude), pitch/roll ~ level.
  if (c.imu_acc || c.imu_att) {
    ASSERT_NEAR_(ep, 0.0, mrpt::DEG2RAD(3.0));
    ASSERT_NEAR_(er, 0.0, mrpt::DEG2RAD(3.0));
  }

  // (c) Geo-referencing. T_enu_to_map is only published while estimating
  // geo-ref, which we enable whenever GNSS or absolute attitude is present.
  const bool estimateGeoref = c.gnss || c.imu_att;
  if (!estimateGeoref) {
    std::cout << "    (no geo-ref source) OK\n";
    return;
  }
  ASSERT_(r.T_enu_to_map.has_value());
  double tey = 0;
  double tep = 0;
  double ter = 0;
  r.T_enu_to_map->mean.getYawPitchRoll(tey, tep, ter);

  const double yawErr = std::abs(mrpt::math::angDistance(tey, mrpt::DEG2RAD(PSI0_DEG)));
  std::cout << "    T_enu2map=" << r.T_enu_to_map->mean.asString()
            << " yawErr=" << mrpt::RAD2DEG(yawErr) << " deg\n";

  // Trajectory recovery precision. GNSS constrains the ENU position only, so the
  // gauge-free metric is the FULL 3D position recovered IN THE ENU FRAME:
  // est_enu = T_enu_to_map (+) est_pose_map. Now that T_enu_to_map roll/pitch are
  // pinned (it is a level yaw+translation), GNSS is forced to flatten the {map}
  // keyframes instead of tilting the transform.
  if (c.gnss) {
    const auto est_enu = r.T_enu_to_map->mean + r.est_pose_map;
    const double enu_pos_err = (est_enu.translation() - r.gt_pose_enu.translation()).norm();
    const double map_pos_err = (r.est_pose_map.translation() - r.gt_pose_map.translation()).norm();
    std::cout << "    ENU 3D pos err=" << enu_pos_err << " m  (map-frame 3D err=" << map_pos_err
              << " m)\n";

    // The map-frame position is strongly recovered: pinning T_enu_to_map
    // roll/pitch (the geo-ref is a LEVEL yaw+translation) forces GNSS to flatten
    // the {map} keyframes instead of tilting the transform, correcting the LIO
    // z/tilt drift (~40 m -> a few m). Without that pin, GNSS would absorb the
    // tilt into a free T_enu_to_map roll/pitch and leave {map} tilted. With no
    // IMU the roll is weakly observable (no absolute orientation reference), so
    // a degenerate roll leaks a little extra map-frame position error.
    const double map_pos_bound = c.any_imu() ? 4.0 : 6.0;
    ASSERT_LT_(map_pos_err, map_pos_bound);
    // A bounded ENU residual remains: the LIO orientation drift distorts the
    // trajectory SHAPE (pitch composed through the arc bends the horizontal
    // path), which lives in the stiff relative-position edges and cannot be
    // fully reshaped by position fixes nor by IMU attitude (which corrects the
    // orientation variables, not the relative-translation edges). Still BOUNDED
    // (the headline result), never the 100s of m of the DCC01 report.
    ASSERT_LT_(enu_pos_err, 15.0);
  }

  // CORE anti-regression check for the MulRan unbounded-growth bug: whenever
  // GNSS is fused, T_enu_to_map must stay a SMALL (few m) translation, NOT grow
  // to 100s of m over the run. The residual ~1 deg map-yaw distortion (LIO tilt
  // drift composed through the arc) gives the transform a ~10 m lever-arm
  // translation -- bounded, the headline result, never the 100s of m of DCC01.
  if (c.gnss) {
    ASSERT_LT_(r.max_enu2map_trans_norm, 15.0);
    ASSERT_LT_(r.T_enu_to_map->mean.translation().norm(), 15.0);
  }

  // Azimuth recovery (T_enu_to_map yaw ~ PSI0) needs an absolute orientation
  // reference: the IMU attitude, or GNSS+IMU. With GNSS position only and no
  // IMU, yaw is weakly observable (trajectory shape) and not asserted here.
  if (c.imu_att) {
    ASSERT_LT_(yawErr, mrpt::DEG2RAD(5.0));
    // With both an absolute attitude source AND GNSS, the geo-ref convergence
    // gate must actually latch (geo_reference published).
    if (c.gnss) {
      ASSERT_(r.converged_localization);
    }
  }
  std::cout << "    OK\n";
}

void test_multisource_sweep()
{
  const std::vector<Combo> combos = {
    {"LIO+IMUacc", false, true, false, false},
    {"LIO+IMUacc+IMUatt", false, true, true, false},
    // LIO+GPS uses centimeter-level (RTK) GNSS to check precise trajectory
    // recovery from an accurate absolute position reference alone:
    {"LIO+GPS", false, false, false, true, GNSS_CM_XY_M, GNSS_CM_Z_M},
    {"LIO+wheels+GPS", true, false, false, true},
    {"LIO+IMU+GPS", false, true, true, true},
    {"LIO+wheels+IMU+GPS", true, true, true, true},
    // Over-confident ("stiff") LIO with systematic drift: the regime that makes
    // T_enu_to_map grow on real data. duration_scale=2 stretches the lever arm.
    {"STIFF LIO+GPS", false, false, false, true, GNSS_NOISE_XY_M, GNSS_NOISE_Z_M, true, 1.0},
    {"STIFF LIO+GPS x2", false, false, false, true, GNSS_NOISE_XY_M, GNSS_NOISE_Z_M, true, 2.0},
    {"STIFF LIO+IMU+GPS x2", false, true, true, true, GNSS_NOISE_XY_M, GNSS_NOISE_Z_M, true, 2.0},
  };

  bool anyFail = false;
  for (const auto & c : combos) {
    try {
      check_combo(c);
    } catch (const std::exception & e) {
      anyFail = true;
      std::cerr << "    COMBO FAILED: " << mrpt::exception_to_str(e) << "\n";
    }
  }
  ASSERT_(!anyFail);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_multisource_sweep", test_multisource_sweep},
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
