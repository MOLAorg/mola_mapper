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
 * @file   test-predict-twist-filter.cpp
 * @brief  Predict-twist low-pass: EMA behavior + end-to-end smoothing.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Ported from mola_state_estimation_smoother's predict-twist filter test.
 */

#include <mola_mapper/Mapper.h>
#include <mola_mapper/WorldModelState.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

using mola::mapper::TwistLowPass;

mrpt::Clock::time_point at(double t) { return mrpt::Clock::fromDouble(t); }

mrpt::math::TTwist3D vx_twist(double vx)
{
  mrpt::math::TTwist3D tw;
  tw.vx = vx;
  return tw;
}

// ---------------------------------------------------------------------------
void test_bootstrap_adopts_raw()
{
  TwistLowPass f;
  ASSERT_(!f.value.has_value());

  f.update(vx_twist(3.0), at(1.0), 0.3);

  ASSERT_(f.value.has_value());
  ASSERT_NEAR_(f.value->vx, 3.0, 1e-12);
}

// ---------------------------------------------------------------------------
// A step input must be approached gradually, at the configured time constant:
// after exactly one tau, an EMA has covered 1 - exp(-1) ~ 63.2% of the step.
void test_ema_step_response()
{
  constexpr double tau = 0.5;

  TwistLowPass f;
  f.update(vx_twist(0.0), at(0.0), tau);
  f.update(vx_twist(1.0), at(tau), tau);

  const double expected = 1.0 - std::exp(-1.0);
  if (VERBOSE) {
    std::cout << "  after 1 tau: " << f.value->vx << " (expected " << expected << ")\n";
  }
  ASSERT_NEAR_(f.value->vx, expected, 1e-9);

  // And it must keep converging, without ever overshooting the step.
  for (int i = 2; i < 50; i++) {
    f.update(vx_twist(1.0), at(tau * i), tau);
    ASSERT_LE_(f.value->vx, 1.0);
  }
  ASSERT_NEAR_(f.value->vx, 1.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Regression: the solver can re-run several times while the newest keyframe
// stamp is unchanged. Overwriting the smoothed state on such a same-stamp
// update wipes the EMA history and lets the raw, un-damped value straight
// through, i.e. exactly the jitter the filter exists to suppress.
void test_same_stamp_preserves_history()
{
  constexpr double tau = 0.3;

  TwistLowPass f;
  f.update(vx_twist(0.0), at(0.0), tau);
  f.update(vx_twist(1.0), at(0.1), tau);

  const double afterFirst = f.value->vx;
  ASSERT_LT_(afterFirst, 1.0);  // damped, not the raw value

  // Same stamp, wildly different raw value: must be ignored entirely.
  f.update(vx_twist(100.0), at(0.1), tau);
  ASSERT_NEAR_(f.value->vx, afterFirst, 1e-12);

  // An out-of-order (older) stamp must be ignored too.
  f.update(vx_twist(100.0), at(0.05), tau);
  ASSERT_NEAR_(f.value->vx, afterFirst, 1e-12);
}

// ---------------------------------------------------------------------------
void test_zero_time_constant_passes_raw_through()
{
  TwistLowPass f;
  f.update(vx_twist(0.0), at(0.0), 0.0);
  f.update(vx_twist(7.0), at(0.1), 0.0);

  ASSERT_NEAR_(f.value->vx, 7.0, 1e-12);
}

// ---------------------------------------------------------------------------
void test_reset_clears_state()
{
  TwistLowPass f;
  f.update(vx_twist(5.0), at(0.0), 0.3);
  ASSERT_(f.value.has_value());

  f.reset();
  ASSERT_(!f.value.has_value());
  ASSERT_(!f.stamp.has_value());

  // ...and it bootstraps again from the next raw value.
  f.update(vx_twist(9.0), at(1.0), 0.3);
  ASSERT_NEAR_(f.value->vx, 9.0, 1e-12);
}

// ---------------------------------------------------------------------------
// The filter must actually reduce the jitter of the velocity that
// estimated_navstate() extrapolates. Feeds a constant-speed trajectory with
// noisy poses and compares the jerk of the predicted velocity, filter on vs
// off.
// ---------------------------------------------------------------------------
double measure_predicted_velocity_jerk(bool filterEnabled)
{
  std::string params =
    R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  odometry_max_insert_rate_hz: 0.0
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  min_time_difference_to_create_new_frame: 0.01
  link_first_pose_to_reference_origin_sigma: 1e-3
)###";
  params +=
    std::string("  predict_twist_filter_enabled: ") + (filterEnabled ? "true" : "false") + "\n";
  params += "  predict_twist_filter_time_const: 0.3\n";

  mola::mapper::Mapper nav;
  nav.initialize(mrpt::containers::yaml::FromText(params));

  // Same noise realization for both runs, so the comparison is apples-to-apples.
  auto & rng = mrpt::random::getRandomGenerator();
  rng.randomize(4242);

  constexpr double T = 0.1;
  constexpr double VX = 1.0;
  constexpr size_t numSteps = 120;
  constexpr const char * ODOM = "odom_lidar";

  mrpt::poses::CPose3D truePose = mrpt::poses::CPose3D::Identity();
  std::vector<double> predictedVx;

  for (size_t i = 1; i <= numSteps; i++) {
    const auto t = at(T * static_cast<double>(i));
    truePose = truePose + mrpt::poses::CPose3D(VX * T, 0, 0, 0, 0, 0);

    // Noisy reported pose: this is what makes the finite-difference velocity
    // jitter in the first place.
    mrpt::poses::CPose3DPDFGaussian pdf;
    pdf.mean = truePose;
    pdf.mean.x_incr(rng.drawGaussian1D(0.0, 0.02));
    pdf.mean.y_incr(rng.drawGaussian1D(0.0, 0.02));
    pdf.cov.setIdentity();
    pdf.cov *= 1e-4;
    nav.fuse_pose(t, pdf, ODOM);

    // Query slightly ahead, the way a front end asks for its ICP prior.
    const auto st = nav.estimated_navstate(at(T * static_cast<double>(i) + 0.5 * T), ODOM);
    if (st.has_value()) {
      predictedVx.push_back(st->twist.vx);
    }
  }

  ASSERT_GT_(predictedVx.size(), 50U);

  // Mean absolute change of the predicted velocity between consecutive queries.
  double jerk = 0;
  for (size_t i = 1; i < predictedVx.size(); i++) {
    jerk += std::abs(predictedVx[i] - predictedVx[i - 1]);
  }
  jerk /= static_cast<double>(predictedVx.size() - 1);

  if (VERBOSE) {
    std::cout << "  filter=" << (filterEnabled ? "on " : "off")
              << " mean |dvx| between queries = " << jerk << "\n";
  }
  return jerk;
}

void test_filter_smooths_the_motion_prior()
{
  const double jerkOff = measure_predicted_velocity_jerk(false);
  const double jerkOn = measure_predicted_velocity_jerk(true);

  ASSERT_GT_(jerkOff, 0.0);
  // The whole point of the filter: a markedly steadier velocity prior.
  ASSERT_LT_(jerkOn, 0.5 * jerkOff);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::map<std::string, std::function<void()>> tests = {
    {"test_bootstrap_adopts_raw", test_bootstrap_adopts_raw},
    {"test_ema_step_response", test_ema_step_response},
    {"test_filter_smooths_the_motion_prior", test_filter_smooths_the_motion_prior},
    {"test_reset_clears_state", test_reset_clears_state},
    {"test_same_stamp_preserves_history", test_same_stamp_preserves_history},
    {"test_zero_time_constant_passes_raw_through", test_zero_time_constant_passes_raw_through},
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
