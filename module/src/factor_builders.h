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
 * @file   factor_builders.h
 * @brief  Internal helpers to emit GTSAM factors into the central-map graph.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Single source of truth for the GTSAM symbol scheme and the kinematic/twist
 * factor construction used by Mapper. Ported from
 * mola_state_estimation_smoother (same factor library, mola_gtsam_factors).
 *
 * Symbol scheme (per keyframe i): T(i)=Pose3 in {map}, V(i)=linear velocity
 * (body), W(i)=angular velocity (body). F(0)=T_enu_to_map; F(k)=
 * T_map_to_odometry_frame_k for k>=1.
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>
#include <mola_gtsam_factors/FactorTricycleKinematic.h>
#include <mola_mapper/Parameters.h>
#include <mrpt/core/exceptions.h>

#include <cstdint>

namespace mola::mapper
{
using gtsam::symbol_shorthand::B;  // IMU bias (accel+gyro)      (imuBias::ConstantBias)
using gtsam::symbol_shorthand::F;  // Frames of reference (Pose3):
                                   // F(0): T_enu_to_map
                                   // F(k): T_map_to_odometry_frame_k (k>=1)
using gtsam::symbol_shorthand::T;  // Keyframe poses             (Pose3)
using gtsam::symbol_shorthand::V;  // Linear velocity (body)     (Point3)
using gtsam::symbol_shorthand::W;  // Angular velocity (body)    (Point3)

/// World/nav-frame ({map}) linear velocity for the IMU-preintegration factor
/// (`gtsam::Vector3`). DISTINCT from the body-frame kinematic velocity V(i):
/// the two models must not alias the same key. Uses symbol char 'u'.
inline gtsam::Key Vw(std::uint64_t i) { return gtsam::Symbol('u', i); }

inline const gtsam::Key symbol_T_enu_to_map = F(0);
inline const gtsam::Key symbol_T_map_to_odom_i_base = F(0);  // odom[k] = base + k (k>=1)

inline constexpr unsigned int REFERENCE_FRAME_ID = 0;  // (for symbol_T_enu_to_map)

inline constexpr double TRICYCLE_LARGE_SIGMAS = 1e6;

/** Adds the constant local-velocity kinematic factors linking keyframes
 *  `from` and `to` separated by `dt` seconds (Eqs (1),(4) of the MOLA RSS2019
 *  paper).
 */
inline void add_const_vel_kinematics(
  gtsam::NonlinearFactorGraph & graph, const Parameters & params, uint64_t from, uint64_t to,
  double dt)
{
  // trick to easily handle queries exactly on an existing keyframe:
  if (dt == 0) {
    dt = 1e-5;
  }
  ASSERT_GT_(dt, 0.);

  const double std_lin_vel = params.sigma_random_walk_acceleration_linear;
  const double std_ang_vel = params.sigma_random_walk_acceleration_angular;

  const auto kTi = T(from);
  const auto kTj = T(to);
  const auto kbVi = V(from);
  const auto kbVj = V(to);
  const auto kbWi = W(from);
  const auto kbWj = W(to);

  graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
    kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

  graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
    kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

  auto noise_kinematicsPosition =
    gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_position);
  auto noise_kinematicsOrientation =
    gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_orientation);

  graph.emplace_shared<mola::factors::FactorTrapezoidalIntegratorPose>(
    kTi, kbVi, kTj, kbVj, dt, noise_kinematicsPosition);

  graph.emplace_shared<mola::factors::FactorAngularVelocityIntegrationPose>(
    kTi, kbWi, kTj, dt, noise_kinematicsOrientation);
}

/** Adds the tricycle kinematic factors linking keyframes `from` and `to`
 *  separated by `dt` seconds.
 */
inline void add_tricycle_kinematics(
  gtsam::NonlinearFactorGraph & graph, const Parameters & params, uint64_t from, uint64_t to,
  double dt)
{
  if (dt == 0) {
    dt = 1e-5;
  }
  ASSERT_GT_(dt, 0.);

  const double std_lin_vel = params.sigma_random_walk_acceleration_linear;
  const double std_ang_vel = params.sigma_random_walk_acceleration_angular;

  const auto kTi = T(from);
  const auto kTj = T(to);
  const auto kbVi = V(from);
  const auto kbVj = V(to);
  const auto kbWi = W(from);
  const auto kbWj = W(to);

  graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
    kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));
  graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
    kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

  // In the tricycle model, body v_y must be zero:
  {
    const Eigen::Vector3d sigmas = {TRICYCLE_LARGE_SIGMAS, std_lin_vel * dt, TRICYCLE_LARGE_SIGMAS};
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
      kbVj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
  }
  // In the tricycle model, body w_x,w_y must be zero:
  {
    const Eigen::Vector3d sigmas = {std_ang_vel * dt, std_ang_vel * dt, TRICYCLE_LARGE_SIGMAS};
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
      kbWj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
  }

  gtsam::Vector6 sigmas;
  const auto sigmaPos = params.sigma_integrator_position;
  const auto sigmaAngle = params.sigma_integrator_orientation;
  sigmas << sigmaAngle, sigmaAngle, sigmaAngle, sigmaPos, sigmaPos, sigmaPos;
  auto noise_kinematics = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  graph.emplace_shared<mola::factors::FactorTricycleKinematic>(
    kTi, kbVi, kbWi, kTj, dt, noise_kinematics);
}

/** Dispatches to the configured kinematic model. */
inline void add_kinematic_factors(
  gtsam::NonlinearFactorGraph & graph, const Parameters & params, uint64_t from, uint64_t to,
  double dt)
{
  switch (params.kinematic_model) {
    case KinematicModel::ConstantVelocity:
      add_const_vel_kinematics(graph, params, from, to, dt);
      break;
    case KinematicModel::Tricycle:
      add_tricycle_kinematics(graph, params, from, to, dt);
      break;
    default:
      THROW_EXCEPTION("Invalid kinematic_model value");
  }
}

/** Adds linear+angular velocity Gaussian priors (twist observation) at `kf`. */
inline void add_twist_priors(
  gtsam::NonlinearFactorGraph & graph, uint64_t kf, const gtsam::Vector3 & v,
  const gtsam::Matrix3 & vCov, const gtsam::Vector3 & w, const gtsam::Matrix3 & wCov)
{
  graph.addPrior(V(kf), v, gtsam::noiseModel::Gaussian::Covariance(vCov));
  graph.addPrior(W(kf), w, gtsam::noiseModel::Gaussian::Covariance(wCov));
}

}  // namespace mola::mapper
