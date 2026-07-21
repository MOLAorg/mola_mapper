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
 * @file   Pose3RelativeRotationFactor.h
 * @brief  Between-keyframe RELATIVE-rotation factor (gyro-preintegrated).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include <array>

namespace mola::mapper
{
/** Constrains the RELATIVE rotation between two keyframe poses `Ti`, `Tj` (in
 *  `{map}`) to a measured value `deltaRij` (the body-frame rotation from frame i
 *  to frame j), obtained by preintegrating the gyroscope over the interval:
 *  `deltaRij = prod Exp(w_k * dt_k)`.
 *
 *  Residual: `Log( deltaRij^{-1} * (R_i^{-1} * R_j) )` in R^3.
 *
 *  Unlike the full IMU preintegration (`CombinedImuFactor` /
 *  `MapFramePreintegratedImuFactor`), this carries NO position/velocity
 *  double-integral and NO gravity vector, so it is robust over the mapper's
 *  long (multi-second) keyframe intervals. It is frame-agnostic: a relative
 *  rotation is unaffected by `{map}` tilt, so it needs neither `F(0)` nor a
 *  gravity-aligned nav frame. Its role is to PROPAGATE the sparse absolute
 *  gravity/attitude anchors forward through the gyro-integrated rotation, which
 *  is exactly what levels the keyframes that are far from any low-dynamics stop
 *  (gravity leveling is an orientation problem).
 *
 *  TODO: upstream to mola_gtsam_factors once validated (kept local while the
 *  design is being tuned on real data).
 *
 *  \ingroup mola_mapper_grp
 */
class Pose3RelativeRotationFactor
  : public gtsam::ExpressionFactorN<
      gtsam::Rot3 /*return type*/, gtsam::Pose3 /*Ti*/, gtsam::Pose3 /*Tj*/>
{
private:
  using This = Pose3RelativeRotationFactor;
  using Base =
    gtsam::ExpressionFactorN<gtsam::Rot3 /*return type*/, gtsam::Pose3 /*Ti*/, gtsam::Pose3 /*Tj*/>;

public:
  Pose3RelativeRotationFactor() = default;

  Pose3RelativeRotationFactor(
    gtsam::Key kTi, gtsam::Key kTj, const gtsam::Rot3 & deltaRij,
    const gtsam::SharedNoiseModel & model, double integratedTime = 0)
  : Base({kTi, kTj}, model, deltaRij), integratedTime_(integratedTime)
  {
    this->initialize(This::expression({kTi, kTj}));
  }

  /// Gyro time [s] integrated into this factor's deltaRij (for diagnostics:
  /// residual / integratedTime gives an implied gyro bias).
  [[nodiscard]] double integratedTime() const { return integratedTime_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::make_shared<This>(*this);
  }

  /// Measurement expression: R_i^{-1} * R_j (rotation of body from i to j).
  gtsam::Expression<gtsam::Rot3> expression(
    const std::array<gtsam::Key, NARY_EXPRESSION_SIZE> & keys) const override
  {
    const gtsam::Expression<gtsam::Pose3> Ti_(keys[0]);
    const gtsam::Expression<gtsam::Pose3> Tj_(keys[1]);
    return gtsam::between(gtsam::rotation(Ti_), gtsam::rotation(Tj_));
  }

private:
  double integratedTime_ = 0;
};

}  // namespace mola::mapper
