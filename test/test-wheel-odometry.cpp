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
 * @file   test-wheel-odometry.cpp
 * @brief  Wheel odometry increments, and their factors in the global graph with
 *         slip segments: of the front end (it falls short) or of the wheels.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GlobalGraph.h>
#include <mola_mapper/GlobalGraphIO.h>
#include <mola_mapper/WheelOdometry.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>

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
using mola::mapper::WheelOdomSample;
using mola::mapper::WheelOdomStatus;
using mrpt::poses::CPose3D;

mrpt::Clock::time_point at(double t) { return mrpt::Clock::fromDouble(1000.0 + t); }

/// Planar motion at constant speed v [m/s] and turn rate w [rad/s].
mrpt::math::TPose2D arc(double t, double v, double w)
{
  if (std::abs(w) < 1e-9) {
    return {v * t, 0, 0};
  }
  return {v / w * std::sin(w * t), v / w * (1 - std::cos(w * t)), mrpt::math::wrapToPi(w * t)};
}

void test_increments()
{
  constexpr double V = 1.0;
  constexpr double W = 0.3;
  std::deque<WheelOdomSample> s;
  for (int i = 0; i <= 200; i++) {
    const double t = 0.05 * i;
    s.push_back({at(t), arc(t, V, W)});
  }
  // Between readings, at both ends:
  mrpt::math::TPose2D inc;
  ASSERT_(
    mola::mapper::wheel_odometry_increment(s, at(2.01), at(3.52), 0.1, inc) == WheelOdomStatus::Ok);
  const auto expected =
    (mrpt::poses::CPose2D(arc(3.52, V, W)) - mrpt::poses::CPose2D(arc(2.01, V, W))).asTPose();
  // Linear interpolation of 0.05 s chords: millimeters.
  ASSERT_LT_(std::hypot(inc.x - expected.x, inc.y - expected.y), 2e-3);
  ASSERT_LT_(std::abs(inc.phi - expected.phi), 1e-6);

  // Not reached yet, older than the readings, and across a gap:
  ASSERT_(
    mola::mapper::wheel_odometry_increment(s, at(9.0), at(10.5), 0.1, inc) ==
    WheelOdomStatus::NotYet);
  ASSERT_(
    mola::mapper::wheel_odometry_increment(s, at(-1.0), at(1.0), 0.1, inc) ==
    WheelOdomStatus::Unavailable);
  auto gap = s;
  gap.erase(gap.begin() + 60, gap.begin() + 70);  // 3.0-3.45 s missing
  ASSERT_(
    mola::mapper::wheel_odometry_increment(gap, at(2.0), at(4.0), 0.1, inc) ==
    WheelOdomStatus::Unavailable);
  ASSERT_(
    mola::mapper::wheel_odometry_increment(gap, at(2.0), at(2.9), 0.1, inc) == WheelOdomStatus::Ok);
  // Exactly at the newest reading:
  ASSERT_(
    mola::mapper::wheel_odometry_increment(s, at(9.0), at(10.0), 0.1, inc) == WheelOdomStatus::Ok);
  std::cout << "Increments: OK\n";
}

/// Ground truth: 120 keyframes 0.5 m apart along a gentle S curve.
std::vector<CPose3D> ground_truth()
{
  std::vector<CPose3D> gt;
  CPose3D p;
  for (int i = 0; i < 120; i++) {
    gt.push_back(p);
    const double turn = (i / 40) % 2 == 0 ? 0.02 : -0.02;
    p = p + CPose3D(0.5, 0, 0, turn, 0, 0);
  }
  return gt;
}

/// Front end and wheel increments from the ground truth, with small noise, and
/// their forward components scaled in [SLIP_FIRST, SLIP_LAST].
constexpr std::size_t SLIP_FIRST = 40;
constexpr std::size_t SLIP_LAST = 70;

GlobalGraphInput make_input(
  const std::vector<CPose3D> & gt, double frontEndScale, double wheelScale)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 1.0);
  GlobalGraphInput in;
  CPose3D odo = gt.front();
  for (std::size_t i = 0; i < gt.size(); i++) {
    GlobalGraphKeyframe kf;
    kf.stamp = at(0.5 * static_cast<double>(i));
    kf.source_frame = "lidar_odom";
    if (i > 0) {
      const bool slip = i >= SLIP_FIRST && i <= SLIP_LAST;
      const CPose3D step = gt[i] - gt[i - 1];
      CPose3D fe = step;
      fe.x(step.x() * (slip ? frontEndScale : 1.0) + 0.002 * noise(rng));
      odo = odo + fe;
      mrpt::math::TPose2D w(
        step.x() * (slip ? wheelScale : 1.0) + 0.01 * noise(rng), step.y() + 0.01 * noise(rng),
        step.yaw());
      kf.wheel_odom_from_prev = w;
      // Gyroscope: exact rotations.
      kf.gyro_rotation_from_prev = CPose3D(0, 0, 0, step.yaw(), step.pitch(), step.roll());
      kf.gyro_rotation_sigma = mrpt::DEG2RAD(0.05);
    }
    kf.pose_in_source.mean = odo;
    kf.pose_in_source.cov.setDiagonal(1e-6);
    kf.initial_guess = odo;
    in.keyframes.push_back(kf);
  }
  return in;
}

double end_error(const std::vector<CPose3D> & est, const std::vector<CPose3D> & gt)
{
  return (est.back().translation() - gt.back().translation()).norm();
}

GlobalGraphParams base_params()
{
  GlobalGraphParams p;
  p.use_gnss = false;
  p.use_imu_gravity = false;
  return p;
}

void test_front_end_slip()
{
  // The front end falls short (degenerate along its motion) by half: 7.75 m.
  const auto gt = ground_truth();
  const auto in = make_input(gt, 0.5, 1.0);

  auto p = base_params();
  p.use_wheel_odometry = false;
  const double errNone = end_error(mola::mapper::optimize_global_graph(in, p).poses, gt);

  // Wheels with the chain edges whole: the stiff chain wins.
  p.use_wheel_odometry = true;
  p.wheel_odom_chain_robust_kernel = "none";
  const auto whole = mola::mapper::optimize_global_graph(in, p);
  const double errWhole = end_error(whole.poses, gt);

  // Default: the forward component of the chain gives way to the wheels.
  p = base_params();
  const auto def = mola::mapper::optimize_global_graph(in, p);
  const double errDefault = end_error(def.poses, gt);

  // A redescending kernel on the wheel factors rejects them, the right ones:
  p.wheel_odom_robust_kernel = "cauchy";
  p.wheel_odom_robust_k = 3.0;
  p.wheel_odom_chain_robust_kernel = "none";
  const double errWheelKernel = end_error(mola::mapper::optimize_global_graph(in, p).poses, gt);

  std::cout << "Front end slip: end error " << errNone << " m without wheels, " << errWhole
            << " m with the chain whole, " << errDefault << " m by default, " << errWheelKernel
            << " m with a Cauchy kernel on the wheels\n";
  ASSERT_EQUAL_(def.num_wheel_odom_factors, gt.size() - 1);
  ASSERT_GT_(errNone, 7.0);
  ASSERT_GT_(errWhole, 5.0);
  ASSERT_LT_(errDefault, 0.3);
  ASSERT_GT_(errWheelKernel, 5.0);
}

void test_wheel_slip()
{
  // The wheels read 50 % more (spinning) while the front end is right.
  const auto gt = ground_truth();
  const auto in = make_input(gt, 1.0, 1.5);

  auto p = base_params();
  p.wheel_odom_robust_kernel = "cauchy";
  p.wheel_odom_robust_k = 3.0;
  p.wheel_odom_chain_robust_kernel = "none";
  const double errRejected = end_error(mola::mapper::optimize_global_graph(in, p).poses, gt);

  // Without a kernel, the wheels drag the stiff chain only a bit:
  p.wheel_odom_robust_kernel = "none";
  const double errGaussian = end_error(mola::mapper::optimize_global_graph(in, p).poses, gt);

  // By default, nothing tells which one slips: the wheels win.
  p = base_params();
  const double errDefault = end_error(mola::mapper::optimize_global_graph(in, p).poses, gt);

  std::cout << "Wheel slip: end error " << errRejected << " m with a Cauchy kernel on the wheels, "
            << errGaussian << " m Gaussian, " << errDefault << " m by default\n";
  ASSERT_LT_(errRejected, 0.2);
  ASSERT_LT_(errRejected, 0.5 * errGaussian);
  ASSERT_GT_(errDefault, 5.0);
}

void test_false_turns()
{
  // Wheels losing encoder counts: false turns (one side), or short readings
  // (both). Those increments are not used.
  const auto gt = ground_truth();
  auto in = make_input(gt, 1.0, 1.0);
  for (std::size_t i = SLIP_FIRST; i <= SLIP_LAST; i++) {
    auto & w = *in.keyframes[i].wheel_odom_from_prev;
    if (i < (SLIP_FIRST + SLIP_LAST) / 2) {
      w.phi += mrpt::DEG2RAD(15.0);
    } else {
      w.x *= 0.4;
    }
  }
  const auto r = mola::mapper::optimize_global_graph(in, base_params());
  const double err = end_error(r.poses, gt);
  std::cout << "False turns: " << r.num_wheel_odom_inconsistent
            << " increments not used, end error " << err << " m\n";
  ASSERT_EQUAL_(r.num_wheel_odom_inconsistent, SLIP_LAST - SLIP_FIRST + 1);
  ASSERT_EQUAL_(r.num_wheel_odom_factors, gt.size() - 1 - r.num_wheel_odom_inconsistent);
  ASSERT_LT_(err, 0.1);
}

void test_io_round_trip()
{
  const auto gt = ground_truth();
  const auto in = make_input(gt, 0.5, 1.0);
  const auto dir = std::filesystem::temp_directory_path() / "mola_mapper_test_wheel_odom";
  std::filesystem::create_directories(dir);
  const std::string prefix = (dir / "graph").string();
  mola::mapper::save_global_graph_input(in, prefix);
  const auto back = mola::mapper::load_global_graph_input(prefix);
  ASSERT_EQUAL_(back.keyframes.size(), in.keyframes.size());
  for (std::size_t i = 0; i < in.keyframes.size(); i++) {
    const auto & a = in.keyframes[i].wheel_odom_from_prev;
    const auto & b = back.keyframes[i].wheel_odom_from_prev;
    ASSERT_EQUAL_(a.has_value(), b.has_value());
    if (a) {
      ASSERT_EQUAL_(a->x, b->x);
      ASSERT_EQUAL_(a->y, b->y);
      ASSERT_EQUAL_(a->phi, b->phi);
    }
  }
  const auto r0 = mola::mapper::optimize_global_graph(in, base_params());
  const auto r1 = mola::mapper::optimize_global_graph(back, base_params());
  ASSERT_LT_(end_error(r0.poses, r1.poses), 1e-6);
  std::filesystem::remove_all(dir);
  std::cout << "IO round trip: OK\n";
}
}  // namespace

int main()
{
  try {
    test_increments();
    test_front_end_slip();
    test_wheel_slip();
    test_false_turns();
    test_io_round_trip();
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
