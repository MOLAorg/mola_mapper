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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_mapper/GlobalGraph.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <cmath>
#include <set>

namespace mola::mapper
{
namespace
{
using gtsam::symbol_shorthand::P;  // P(i): vehicle pose of keyframe i, in {map}
using gtsam::symbol_shorthand::T;  // T(0): T_enu_to_map

/// Sigmas of an odometry edge, in GTSAM Pose3 tangent order (Rx, Ry, Rz, tx, ty, tz).
gtsam::Vector6 odometry_edge_sigmas(
  const mrpt::poses::CPose3DPDFGaussian & rel, const GlobalGraphParams & p)
{
  // MRPT covariance order: x y z yaw pitch roll
  gtsam::Vector6 s = rel.cov.asEigen().diagonal().cwiseMax(0.0).array().sqrt().eval();
  s *= p.odometry_uncertainty_multiplier;

  const double sqrtStep = std::sqrt(rel.mean.translation().norm());
  for (int k = 0; k < 3; k++) {
    s[k] += p.odometry_additional_noise_xyz + p.odometry_noise_xyz_per_sqrt_m * sqrtStep;
    s[3 + k] += mrpt::DEG2RAD(
      p.odometry_additional_noise_ang_deg + p.odometry_noise_ang_deg_per_sqrt_m * sqrtStep);
  }
  gtsam::Vector6 sigmas;
  sigmas << s[5], s[4], s[3], s[0], s[1], s[2];
  return sigmas;
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
      fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        P(i - 1), P(i), mrpt::gtsam_wrappers::toPose3(rel.mean),
        gtsam::noiseModel::Diagonal::Sigmas(odometry_edge_sigmas(rel, params)));
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
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(pdf, rel, relCov);
    auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params.loop_closure_robust_huber_k),
      gtsam::noiseModel::Gaussian::Covariance(relCov));
    fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(P(lc.from), P(lc.to), rel, robust);
    out.num_loop_factors++;
  }
}

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
    mrpt::poses::CPose3DPDFGaussian pdf;
    pdf.mean = out.poses.back();
    pdf.cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(*lastCov);
    out.last_pose = pdf;
  }

  if (out.num_gnss_factors > 0 && out.geodetic_datum) {
    mola::Georeferencing g;
    g.geo_coord = *out.geodetic_datum;
    g.T_enu_to_map.mean =
      mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(result.at<gtsam::Pose3>(T(0))));
    if (enuCov) {
      g.T_enu_to_map.cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(*enuCov);
    }
    out.georeferencing = g;
  }

  if (logger != nullptr) {
    logger->logFmt(
      mrpt::system::LVL_DEBUG,
      "[GlobalGraph] %zu keyframes, %zu odometry, %zu loop, %zu GNSS%s, %zu gravity factors: "
      "error %.3e -> %.3e",
      N, out.num_odometry_factors, out.num_loop_factors, out.num_gnss_factors,
      out.gnss_degenerate ? " (degenerate, unused)" : "", out.num_gravity_factors,
      out.initial_error, out.final_error);
  }
  return out;
}

}  // namespace mola::mapper
