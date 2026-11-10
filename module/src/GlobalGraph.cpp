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
 * @file   GlobalGraph.cpp
 * @brief  Batch pose graph over all the keyframes of the central map.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_mapper/GlobalGraph.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "mrpt_gtsam_compat.h"

namespace mola::mapper
{
namespace
{
using gtsam::symbol_shorthand::P;  // P(i): vehicle pose of keyframe i, in {map}
using gtsam::symbol_shorthand::T;  // T(0): T_enu_to_map

/// Sigma of the components a factor leaves unconstrained [m].
constexpr double FREE_SIGMA = 1e3;

gtsam::noiseModel::mEstimator::Base::shared_ptr robust_kernel(const std::string & name, double k)
{
  namespace m = gtsam::noiseModel::mEstimator;
  if (name == "huber") {
    return m::Huber::Create(k);
  }
  if (name == "cauchy") {
    return m::Cauchy::Create(k);
  }
  if (name == "geman_mcclure") {
    return m::GemanMcClure::Create(k);
  }
  if (name == "tukey") {
    return m::Tukey::Create(k);
  }
  THROW_EXCEPTION_FMT("Unknown robust kernel '%s'", name.c_str());
}

/// The wheel odometry increment of keyframe kf (prev: the one before it), if
/// there is one and it is usable: wheels that lose encoder counts read false
/// turns, and less motion than the front end (which, when degenerate, reads less
/// than the real one, not more), along its direction of travel (forward or
/// reverse).
const mrpt::math::TPose2D * usable_wheel_odom(
  const GlobalGraphKeyframe & prev, const GlobalGraphKeyframe & kf, const GlobalGraphParams & p)
{
  if (!p.use_wheel_odometry || !kf.wheel_odom_from_prev) {
    return nullptr;
  }
  if (prev.source_frame == kf.source_frame) {
    const double frontEndForward = (kf.pose_in_source.mean - prev.pose_in_source.mean).x();
    // Below the floor the direction of travel is unknown: no check.
    if (std::abs(frontEndForward) > p.wheel_odom_ratio_floor) {
      const double along =
        frontEndForward > 0 ? kf.wheel_odom_from_prev->x : -kf.wheel_odom_from_prev->x;
      if (
        along < p.wheel_odom_min_ratio_to_front_end * std::abs(frontEndForward) -
                  p.wheel_odom_ratio_floor) {
        return nullptr;
      }
    }
  }
  if (kf.gyro_rotation_from_prev) {
    const double gyroYaw = kf.gyro_rotation_from_prev->yaw();
    const double err = std::abs(mrpt::math::wrapToPi(kf.wheel_odom_from_prev->phi - gyroYaw));
    const double maxErr = mrpt::DEG2RAD(p.wheel_odom_max_yaw_error_deg) +
                          p.wheel_odom_max_yaw_error_ratio * std::abs(gyroYaw);
    if (err > maxErr) {
      return nullptr;
    }
  }
  return &*kf.wheel_odom_from_prev;
}

/// Sigmas of an odometry edge, from the (inflated) diagonal of its MRPT covariance.
gtsam::Vector6 odometry_edge_sigmas(
  const mrpt::poses::CPose3DPDFGaussian & rel, const GlobalGraphParams & p)
{
  // MRPT covariance order: x y z yaw pitch roll
  gtsam::Vector6 s = rel.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();
  s *= p.odometry_uncertainty_multiplier;

  const double sqrtStep = std::sqrt(rel.mean.translation().norm());
  const double turned = mrpt::poses::Lie::SO<3>::log(rel.mean.getRotationMatrix()).norm();
  for (int k = 0; k < 3; k++) {
    s[k] += p.odometry_additional_noise_xyz + p.odometry_noise_xyz_per_sqrt_m * sqrtStep;
    s[3 + k] +=
      mrpt::DEG2RAD(
        p.odometry_additional_noise_ang_deg + p.odometry_noise_ang_deg_per_sqrt_m * sqrtStep) +
      p.odometry_noise_ang_per_rad * turned;
  }
  return s;
}

/// Noise model of an odometry edge from its sigmas, in MRPT layout.
gtsam::SharedNoiseModel odometry_edge_noise(
  const mrpt::poses::CPose3D & relMean, const gtsam::Vector6 & s)
{
  // The sigmas are in MRPT layout (global translation, yaw/pitch/roll): convert
  // them into the GTSAM tangent of the increment, rotation included.
  mrpt::poses::CPose3DPDFGaussian inflated;
  inflated.mean = relMean;
  inflated.cov.setZero();
  for (int k = 0; k < 6; k++) {
    inflated.cov(k, k) = s[k] * s[k];
  }
  gtsam::Pose3 unused;
  gtsam::Matrix6 cov;
  mola::gtsam_compat::to_gtsam_se3_cov6(inflated, unused, cov);
  return gtsam::noiseModel::Gaussian::Covariance(cov);
}

void add_odometry_factors(
  gtsam::NonlinearFactorGraph & fg, const GlobalGraphInput & in, const GlobalGraphParams & params,
  GlobalGraphResult & out)
{
  for (std::size_t i = 1; i < in.keyframes.size(); i++) {
    const auto & prev = in.keyframes[i - 1];
    const auto & cur = in.keyframes[i];
    if (prev.source_frame == cur.source_frame) {
      const mrpt::poses::CPose3DPDFGaussian rel = cur.pose_in_source - prev.pose_in_source;
      gtsam::Vector6 s = odometry_edge_sigmas(rel, params);
      // With wheel odometry, the forward component goes into a factor of its
      // own, with a robust kernel, so the wheels win where the front end slips:
      const bool robustForward = usable_wheel_odom(prev, cur, params) != nullptr &&
                                 params.wheel_odom_chain_robust_kernel != "none";
      if (robustForward) {
        const gtsam::Vector3 fs(s[0], FREE_SIGMA, FREE_SIGMA);
        s[0] = FREE_SIGMA;
        const auto model = gtsam::noiseModel::Robust::Create(
          robust_kernel(params.wheel_odom_chain_robust_kernel, params.wheel_odom_chain_robust_k),
          gtsam::noiseModel::Diagonal::Sigmas(fs));
        const gtsam::Pose3_ a(P(i - 1));
        const gtsam::Pose3_ b(P(i));
        fg.emplace_shared<gtsam::ExpressionFactor<gtsam::Point3>>(
          model, gtsam::Point3(rel.mean.x(), rel.mean.y(), rel.mean.z()),
          gtsam::transformTo(a, gtsam::translation(b)));
      }
      fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        P(i - 1), P(i), mrpt::gtsam_wrappers::toPose3(rel.mean), odometry_edge_noise(rel.mean, s));
    } else {
      // A different source (e.g. the front end restarted): keep the graph connected
      // with a weak edge at the initial guesses.
      const auto rel = cur.initial_guess - prev.initial_guess;
      fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        P(i - 1), P(i), mrpt::gtsam_wrappers::toPose3(rel),
        gtsam::noiseModel::Isotropic::Sigma(6, 10.0));
    }
    out.num_odometry_factors++;
  }
}

/// Gyroscope rotations between consecutive keyframes: a relative pose factor
/// with an unconstrained translation.
void add_gyro_factors(
  gtsam::NonlinearFactorGraph & fg, const GlobalGraphInput & in, const GlobalGraphParams & params,
  GlobalGraphResult & out)
{
  if (!params.use_gyro) {
    return;
  }
  for (std::size_t i = 1; i < in.keyframes.size(); i++) {
    const auto & cur = in.keyframes[i];
    if (!cur.gyro_rotation_from_prev) {
      continue;
    }
    const double sigma =
      std::max(cur.gyro_rotation_sigma, mrpt::DEG2RAD(params.gyro_min_sigma_deg));
    gtsam::Vector6 sigmas;
    sigmas << sigma, sigma, sigma, FREE_SIGMA, FREE_SIGMA, FREE_SIGMA;
    fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      P(i - 1), P(i), mrpt::gtsam_wrappers::toPose3(*cur.gyro_rotation_from_prev),
      gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    out.num_gyro_factors++;
  }
}

/// Wheel odometry between consecutive keyframes: the translation of the relative
/// pose in the frame of the first one, forward and lateral (its vertical
/// component left free).
void add_wheel_odometry_factors(
  gtsam::NonlinearFactorGraph & fg, const GlobalGraphInput & in, const GlobalGraphParams & params,
  GlobalGraphResult & out)
{
  if (!params.use_wheel_odometry) {
    return;
  }
  for (std::size_t i = 1; i < in.keyframes.size(); i++) {
    const auto * inc = usable_wheel_odom(in.keyframes[i - 1], in.keyframes[i], params);
    if (in.keyframes[i].wheel_odom_from_prev && !inc) {
      out.num_wheel_odom_inconsistent++;
    }
    if (!inc) {
      continue;
    }
    const double dist = std::hypot(inc->x, inc->y);
    const double common =
      params.wheel_odom_min_sigma + params.wheel_odom_sigma_per_rad * std::abs(inc->phi);
    const gtsam::Vector3 sigmas(
      common + params.wheel_odom_sigma_per_m * dist,
      common + params.wheel_odom_lateral_sigma_per_m * dist, FREE_SIGMA);
    gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    if (params.wheel_odom_robust_kernel != "none") {
      model = gtsam::noiseModel::Robust::Create(
        robust_kernel(params.wheel_odom_robust_kernel, params.wheel_odom_robust_k), model);
    }
    const gtsam::Pose3_ a(P(i - 1));
    const gtsam::Pose3_ b(P(i));
    fg.emplace_shared<gtsam::ExpressionFactor<gtsam::Point3>>(
      model, gtsam::Point3(inc->x, inc->y, 0), gtsam::transformTo(a, gtsam::translation(b)));
    out.num_wheel_odom_factors++;
  }
}

void add_loop_factors(
  gtsam::NonlinearFactorGraph & fg, const GlobalGraphInput & in, const GlobalGraphParams & params,
  GlobalGraphResult & out)
{
  const std::size_t N = in.keyframes.size();
  for (const auto & lc : in.loops) {
    if (lc.from >= N || lc.to >= N || lc.from == lc.to) {
      continue;
    }
    gtsam::Pose3 rel;
    gtsam::Matrix6 relCov;
    auto pdf = lc.relative_pose;
    for (int k = 0; k < 6; k++) {
      const double floor = k < 3 ? params.loop_closure_min_sigma_xyz
                                 : mrpt::DEG2RAD(params.loop_closure_min_sigma_ang_deg);
      pdf.cov(k, k) = std::max(pdf.cov(k, k), floor * floor);
    }
    mola::gtsam_compat::to_gtsam_se3_cov6(pdf, rel, relCov);
    auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params.loop_closure_robust_huber_k),
      gtsam::noiseModel::Gaussian::Covariance(relCov));
    fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(P(lc.from), P(lc.to), rel, robust);
    out.num_loop_factors++;
  }
}

void add_gnss_factors(
  gtsam::NonlinearFactorGraph & fg, const mrpt::maps::CSimpleMap & auxMap,
  const GlobalGraphInput & in, const GlobalGraphParams & params, GlobalGraphResult & out)
{
  const auto gnss = mola::extract_gnss_frames_from_sm(auxMap, in.geodetic_datum);
  if (!out.geodetic_datum && gnss.refCoord) {
    out.geodetic_datum = gnss.refCoord;
  }
  out.gnss_degenerate = gnss.possibly_degenerate;
  if (gnss.possibly_degenerate) {
    return;
  }
  for (const auto & f : gnss.frames) {
    if (std::hypot(f.sigma_E, f.sigma_N) > params.gnss_max_uncertainty_horiz) {
      continue;
    }
    const gtsam::Vector3 sigmas =
      (gtsam::Vector3(f.sigma_E, f.sigma_N, f.sigma_U) * params.gnss_uncertainty_multiplier)
        .array()
        .max(params.gnss_minimum_uncertainty);
    auto robust = gtsam::noiseModel::Robust::Create(
      robust_kernel(params.gnss_robust_kernel, params.gnss_robust_k),
      gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    fg.emplace_shared<mola::factors::FactorGnssMapEnu>(
      T(0), P(f.kf_index), mrpt::gtsam_wrappers::toPoint3(f.obs->sensorPose.translation()),
      mrpt::gtsam_wrappers::toPoint3(f.enu), robust);
    out.num_gnss_factors++;
  }
}

void add_gravity_factors(
  gtsam::NonlinearFactorGraph & fg, gtsam::Values & values, const mrpt::maps::CSimpleMap & auxMap,
  const GlobalGraphParams & params, GlobalGraphResult & out)
{
  const auto imuFrames = mola::extract_imu_frames_from_sm(auxMap);
  // All the keyframes already have their P(i), so the helper adds no new variable:
  std::set<std::size_t> allKeys;
  for (std::size_t i = 0; i < auxMap.size(); i++) {
    allKeys.insert(i);
  }
  mola::AddIMUGravityFactorParams gp;
  gp.imuGravitySigmaDeg = params.imu_gravity_sigma_deg;
  const auto before = fg.size();
  mola::add_imu_gravity_factors(fg, values, imuFrames, allKeys, gp);
  out.num_gravity_factors = fg.size() - before;
}

}  // namespace

GlobalGraphResult optimize_global_graph(
  const GlobalGraphInput & in, const GlobalGraphParams & params,
  mrpt::system::COutputLogger * logger)
{
  GlobalGraphResult out;
  out.geodetic_datum = in.geodetic_datum;

  const std::size_t N = in.keyframes.size();
  if (N == 0) {
    return out;
  }

  gtsam::NonlinearFactorGraph fg;
  gtsam::Values values;

  for (std::size_t i = 0; i < N; i++) {
    values.insert(P(i), mrpt::gtsam_wrappers::toPose3(in.keyframes[i].initial_guess));
  }

  // Gauge: the first keyframe stays where it is, so {map} keeps its meaning.
  fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    P(0), mrpt::gtsam_wrappers::toPose3(in.keyframes[0].initial_guess),
    gtsam::noiseModel::Isotropic::Sigma(6, params.first_pose_prior_sigma));

  add_odometry_factors(fg, in, params, out);
  add_loop_factors(fg, in, params, out);
  add_wheel_odometry_factors(fg, in, params, out);
  add_gyro_factors(fg, in, params, out);

  // GNSS and gravity, from the observations attached to the keyframes, through
  // the extractors of mola_georeferencing (they read a CSimpleMap):
  values.insert(
    T(0), mrpt::gtsam_wrappers::toPose3(
            in.T_enu_to_map_guess.value_or(mrpt::poses::CPose3D::Identity())));

  mrpt::maps::CSimpleMap auxMap;
  for (const auto & kf : in.keyframes) {
    auto sf = kf.aux_observations
                ? std::const_pointer_cast<mrpt::obs::CSensoryFrame>(kf.aux_observations)
                : mrpt::obs::CSensoryFrame::Create();
    auxMap.insert(mrpt::poses::CPose3DPDFGaussian::Create(kf.initial_guess), sf);
  }

  if (params.use_gnss) {
    add_gnss_factors(fg, auxMap, in, params, out);
  }
  if (params.use_imu_gravity) {
    add_gravity_factors(fg, values, auxMap, params, out);
  }

  // T_enu_to_map: without GNSS, only its roll and pitch are observed (by gravity),
  // so a weak prior keeps the rest defined. With GNSS, a very weak one only for
  // numerical safety (e.g. GNSS fixes on a straight line).
  {
    const double rotSigma = out.num_gnss_factors > 0 ? 10.0 : 1.0;
    const double posSigma = out.num_gnss_factors > 0 ? 1e4 : 1e3;
    gtsam::Vector6 s;
    s << rotSigma, rotSigma, rotSigma, posSigma, posSigma, posSigma;
    fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      T(0), values.at<gtsam::Pose3>(T(0)), gtsam::noiseModel::Diagonal::Sigmas(s));
  }

  gtsam::LevenbergMarquardtParams lmParams;
  lmParams.maxIterations = params.max_iterations;
  out.initial_error = fg.error(values);
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(fg, values, lmParams).optimize();
  out.final_error = fg.error(result);

  out.poses.reserve(N);
  for (std::size_t i = 0; i < N; i++) {
    out.poses.emplace_back(mrpt::gtsam_wrappers::toTPose3D(result.at<gtsam::Pose3>(P(i))));
  }

  std::optional<gtsam::Matrix6> lastCov;
  std::optional<gtsam::Matrix6> enuCov;
  try {
    const gtsam::Marginals marginals(fg, result);
    lastCov = marginals.marginalCovariance(P(N - 1));
    enuCov = marginals.marginalCovariance(T(0));
  } catch (const std::exception & e) {
    if (logger != nullptr) {
      logger->logFmt(mrpt::system::LVL_WARN, "[GlobalGraph] marginals failed: %s", e.what());
    }
  }
  if (lastCov) {
    out.last_pose =
      mola::gtsam_compat::to_mrpt_se3_pdf(result.at<gtsam::Pose3>(P(N - 1)), *lastCov);
  }

  if (out.num_gnss_factors >= params.georef_min_gnss_factors && out.geodetic_datum && enuCov) {
    mola::Georeferencing g;
    g.geo_coord = *out.geodetic_datum;
    g.T_enu_to_map = mola::gtsam_compat::to_mrpt_se3_pdf(result.at<gtsam::Pose3>(T(0)), *enuCov);
    // Observable once the azimuth is: a single fix, or fixes close together for
    // their sigmas (standing still), leave it undetermined.
    // MRPT covariance order: x y z yaw pitch roll
    const double sigmaYaw = std::sqrt(g.T_enu_to_map.cov(3, 3));
    out.georef_sigma_yaw = sigmaYaw;
    if (sigmaYaw <= mrpt::DEG2RAD(params.georef_max_sigma_yaw_deg)) {
      out.georeferencing = g;
    }
  }

  if (logger != nullptr) {
    logger->logFmt(
      mrpt::system::LVL_DEBUG,
      "[GlobalGraph] %zu keyframes, %zu odometry, %zu loop, %zu GNSS%s, %zu gravity, %zu gyro, "
      "%zu wheel odometry factors (%zu inconsistent, not used): error %.3e -> %.3e",
      N, out.num_odometry_factors, out.num_loop_factors, out.num_gnss_factors,
      out.gnss_degenerate ? " (degenerate, unused)" : "", out.num_gravity_factors,
      out.num_gyro_factors, out.num_wheel_odom_factors, out.num_wheel_odom_inconsistent,
      out.initial_error, out.final_error);
  }
  return out;
}

}  // namespace mola::mapper
