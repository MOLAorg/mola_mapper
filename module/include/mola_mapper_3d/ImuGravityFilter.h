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
 * @file   ImuGravityFilter.h
 * @brief  Filtered gravity-direction extraction from a raw high-rate IMU stream.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/math/TPoint3D.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace mola::mapper_3d
{
/** Accumulates a short window of raw IMU samples and extracts ONE filtered,
 *  low-dynamics gravity-direction estimate per window.
 *
 *  Rationale (see agents.md "IMU gravity leveling"): a single accelerometer
 *  sample measures specific force = gravity + body acceleration, so the
 *  per-sample "up" direction is contaminated by vehicle dynamics (a few degrees,
 *  up to >10 deg in hard motion) in RANDOM directions. Emitting one
 *  `MeasuredGravityFactor` per raw/summarized sample therefore feeds the graph a
 *  zero-mean-but-noisy "up", which can only pin the map's MEAN tilt to
 *  ~noise/sqrt(N) and never bends the keyframe chain enough to level it.
 *
 *  This filter instead:
 *   - rejects samples whose specific-force magnitude departs from gravity
 *     (|‖a‖-g| > accel_tol_frac*g) or whose angular rate is high (‖w‖ >
 *     gyro_tol_dps), i.e. keeps only near-static / constant-velocity epochs;
 *   - robustly averages the surviving samples (1 IRLS Huber pass) into a single
 *     body-frame gravity direction;
 *   - reports an EARNED 1-sigma (deg) from the angular spread of the accepted
 *     samples divided by sqrt(n) (clamped to [floor, ceil]), so the resulting
 *     factor is trusted in proportion to how many clean, agreeing samples backed
 *     it. With a 100 Hz IMU and a ~1 s window that is ~100 samples, shrinking the
 *     ~2 deg per-sample noise to a few tenths of a degree.
 *
 *  Thread-safety: none; the caller serializes access (Mapper3D holds it under
 *  stateMutex_).
 *
 *  \ingroup mola_mapper_3d_grp
 */
class ImuGravityFilter
{
public:
  struct Parameters
  {
    /// Emit one estimate roughly every this many seconds of accumulated data.
    double window_sec = 1.0;
    /// Accept a sample only if | ‖a‖ - g | <= accel_tol_frac * g.
    double accel_tol_frac = 0.05;
    /// Accept a sample only if its angular rate ‖w‖ <= this many deg/s.
    double gyro_tol_dps = 3.0;
    /// Do not emit an estimate from fewer than this many accepted samples.
    std::size_t min_accepted = 10;
    /// Lower clamp on the earned 1-sigma [deg] (sensor/quantization floor).
    double sigma_floor_deg = 0.5;
    /// Upper clamp on the earned 1-sigma [deg] (above this the window is noisy).
    double sigma_ceil_deg = 5.0;
    /// Expected gravity magnitude [m/s^2] (used for the magnitude gate).
    double expected_g = 9.81;
    /// Huber threshold on the per-sample residual to the running mean [m/s^2].
    double huber_threshold = 0.5;
  };

  struct Estimate
  {
    /// Normalized filtered gravity direction in the IMU/body frame.
    mrpt::math::TVector3D gravity_body_normalized{0, 0, 0};
    /// Earned 1-sigma [deg] for a MeasuredGravityFactor built from this.
    double sigma_deg = 0;
    std::size_t n_accepted = 0;
    std::size_t n_total = 0;
    /// Representative timestamp (last sample in the window).
    mrpt::Clock::time_point stamp{};
  };

  ImuGravityFilter() = default;
  explicit ImuGravityFilter(Parameters p) : params_(p) {}

  const Parameters & params() const { return params_; }
  void setParams(const Parameters & p) { params_ = p; }

  /** Adds one raw IMU sample. `acc_body` is the specific force [m/s^2] in the
   *  IMU frame; `w_body` is the optional angular rate [rad/s] in the IMU frame.
   */
  void addSample(
    const mrpt::Clock::time_point & stamp, const mrpt::math::TVector3D & acc_body,
    const std::optional<mrpt::math::TVector3D> & w_body);

  /** @returns true once the accumulated window spans >= window_sec. */
  [[nodiscard]] bool windowReady(const mrpt::Clock::time_point & now) const;

  /** Computes a filtered estimate from the accumulated window and CLEARS it.
   *  @returns nullopt if fewer than `min_accepted` samples passed the gates.
   */
  [[nodiscard]] std::optional<Estimate> flush();

  void clear();

  [[nodiscard]] std::size_t bufferedSamples() const { return acc_.size(); }

private:
  Parameters params_;

  struct RawSample
  {
    mrpt::Clock::time_point stamp;
    mrpt::math::TVector3D acc;
    bool accepted = false;
  };
  std::vector<RawSample> acc_;
  std::optional<mrpt::Clock::time_point> window_start_;
};

}  // namespace mola::mapper_3d
