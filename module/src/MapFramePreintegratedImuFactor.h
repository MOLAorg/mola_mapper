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
 * @file   MapFramePreintegratedImuFactor.h
 * @brief  IMU preintegration factor over {map} variables with gravity in {enu}.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

namespace mola::mapper
{
/** Combined IMU-preintegration factor ("Option A") whose keyframe pose/velocity
 *  variables live in the (possibly non-level) `{map}` frame, but whose gravity
 *  is expressed EXACTLY in the gravity-aligned `{enu}` frame by composing the
 *  persistent `F(0) = T_enu_to_map` variable inside the factor.
 *
 *  This is the correct fix for the failure mode of the stock GTSAM
 *  `CombinedImuFactor` used as "Option B": there gravity is baked as a constant
 *  `R_enu_to_map * (0,0,-g)` in `{map}`, which is only right while `{map}` is
 *  gravity-aligned. Without an absolute-attitude anchor (raw accel+gyro IMU),
 *  `{map}` tilts with drift and the stale gravity gets double-integrated into
 *  meters of error (validated on Oxford Spires). Composing `F(0)` makes gravity
 *  exact regardless of `{map}` tilt.
 *
 *  7 keys: F(0)=T_enu_to_map (map->enu, i.e. p_enu = F0 * p_map, matching the
 *  MeasuredGravityFactor convention), T(i), Vw(i), T(j), Vw(j), B(i), B(j),
 *  where T are `{map}` poses and Vw are `{map}` (world) velocities. The 15-dim
 *  residual mirrors gtsam::CombinedImuFactor: [9-dim preintegration error via
 *  PreintegrationBase::computeError in {enu}] ++ [6-dim bias random walk].
 *
 *  The `PreintegratedCombinedMeasurements` passed in MUST be built with gravity
 *  = (0,0,-g) in `{enu}` (identity rotation), NOT the `{map}` gravity.
 *
 *  Jacobians are numerical (the feature is experimental / off by default);
 *  swap for analytic ones if it becomes a hot path.
 *
 *  \ingroup mola_mapper_grp
 */
class MapFramePreintegratedImuFactor : public gtsam::NoiseModelFactor
{
public:
  using Base = gtsam::NoiseModelFactor;

  MapFramePreintegratedImuFactor(
    gtsam::Key kT_enu_to_map, gtsam::Key kTi, gtsam::Key kVwi, gtsam::Key kTj, gtsam::Key kVwj,
    gtsam::Key kBi, gtsam::Key kBj, const gtsam::PreintegratedCombinedMeasurements & pim)
  : Base(
      gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
      std::vector<gtsam::Key>{kT_enu_to_map, kTi, kVwi, kTj, kVwj, kBi, kBj}),
    pim_(pim)
  {
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::make_shared<MapFramePreintegratedImuFactor>(*this);
  }

  gtsam::Vector unwhitenedError(
    const gtsam::Values & x,
    boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
  {
    const gtsam::Pose3 F0 = x.at<gtsam::Pose3>(keys()[0]);
    const gtsam::Pose3 Ti = x.at<gtsam::Pose3>(keys()[1]);
    const gtsam::Vector3 Vwi = x.at<gtsam::Vector3>(keys()[2]);
    const gtsam::Pose3 Tj = x.at<gtsam::Pose3>(keys()[3]);
    const gtsam::Vector3 Vwj = x.at<gtsam::Vector3>(keys()[4]);
    const gtsam::imuBias::ConstantBias Bi = x.at<gtsam::imuBias::ConstantBias>(keys()[5]);
    const gtsam::imuBias::ConstantBias Bj = x.at<gtsam::imuBias::ConstantBias>(keys()[6]);

    if (H) {
      H->resize(7);
      (*H)[0] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::Pose3>(
        [&](const gtsam::Pose3 & v) { return residual(v, Ti, Vwi, Tj, Vwj, Bi, Bj); }, F0);
      (*H)[1] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::Pose3>(
        [&](const gtsam::Pose3 & v) { return residual(F0, v, Vwi, Tj, Vwj, Bi, Bj); }, Ti);
      (*H)[2] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::Vector3>(
        [&](const gtsam::Vector3 & v) { return residual(F0, Ti, v, Tj, Vwj, Bi, Bj); }, Vwi);
      (*H)[3] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::Pose3>(
        [&](const gtsam::Pose3 & v) { return residual(F0, Ti, Vwi, v, Vwj, Bi, Bj); }, Tj);
      (*H)[4] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::Vector3>(
        [&](const gtsam::Vector3 & v) { return residual(F0, Ti, Vwi, Tj, v, Bi, Bj); }, Vwj);
      (*H)[5] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::imuBias::ConstantBias>(
        [&](const gtsam::imuBias::ConstantBias & v) {
          return residual(F0, Ti, Vwi, Tj, Vwj, v, Bj);
        },
        Bi);
      (*H)[6] = gtsam::numericalDerivative11<gtsam::Vector15, gtsam::imuBias::ConstantBias>(
        [&](const gtsam::imuBias::ConstantBias & v) {
          return residual(F0, Ti, Vwi, Tj, Vwj, Bi, v);
        },
        Bj);
    }
    return residual(F0, Ti, Vwi, Tj, Vwj, Bi, Bj);
  }

private:
  gtsam::PreintegratedCombinedMeasurements pim_;

  /// The 15-dim residual: express the {map} keyframe states in the
  /// gravity-aligned {enu} frame (p_enu = F0 * p_map, v_enu = R(F0) * v_map),
  /// then reuse the standard preintegration + bias-random-walk error.
  gtsam::Vector15 residual(
    const gtsam::Pose3 & F0, const gtsam::Pose3 & Ti, const gtsam::Vector3 & Vwi,
    const gtsam::Pose3 & Tj, const gtsam::Vector3 & Vwj, const gtsam::imuBias::ConstantBias & Bi,
    const gtsam::imuBias::ConstantBias & Bj) const
  {
    const gtsam::Rot3 Rem = F0.rotation();
    const gtsam::NavState state_i(F0.compose(Ti), Rem * Vwi);
    const gtsam::NavState state_j(F0.compose(Tj), Rem * Vwj);

    gtsam::Vector15 e;
    e.head<9>() = pim_.computeError(state_i, state_j, Bi, boost::none, boost::none, boost::none);
    e.tail<6>() = gtsam::traits<gtsam::imuBias::ConstantBias>::Between(Bj, Bi).vector();
    return e;
  }
};

}  // namespace mola::mapper
