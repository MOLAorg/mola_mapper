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
 * @file   test-lc-wait-idle.cpp
 * @brief  wait_for_loop_closure_idle(): turning fire-and-forget loop closure
 *         into something a caller can act on the result of.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * request_loop_closure_scan() returns immediately and the scan lands whenever
 * the background thread gets to it. That is fine for a GUI button and useless
 * for a test, a save, or a run being compared against another run. These cases
 * pin the contract of the wait that closes that gap.
 *
 * No real loop closure happens here, deliberately: the keyframes carry no point
 * clouds, so the detector generates zero candidates and returns. What is under
 * test is the HANDSHAKE -- that the wait covers the whole request-to-completion
 * window and does not return in the gap between them -- not the ICP.
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

using namespace mrpt::literals;  // _deg

namespace
{
std::string write_minimal_lc_pipeline()
{
  const std::string fil = mrpt::system::getTempFileName() + "_lc.yaml";
  std::ofstream f(fil);
  ASSERT_(f.is_open());
  // Enough for the engine to initialize. It never reaches ICP here: with no
  // point clouds on the keyframes, candidate generation yields nothing.
  f << "params:\n"
    << "  min_frames_between_lc: 1\n"
    << "  max_distance_for_lc_candidate: 5.0\n"
    << "  profiler_enabled: false\n"
    << "  save_trajectory_files: false\n"
    // Structurally required by the engine even though it is never exercised
    // here (zero candidates means ICP is never reached).
    << "icp_settings:\n"
    << "  class_name: mp2p_icp::ICP\n"
    << "  params:\n"
    << "    maxIterations: 1\n"
    << "  solvers:\n"
    << "    - class: mp2p_icp::Solver_GaussNewton\n"
    << "      params:\n"
    << "        maxIterations: 1\n"
    << "  matchers:\n"
    << "    - class: mp2p_icp::Matcher_Points_DistanceThreshold\n"
    << "      params:\n"
    << "        threshold: 1.0\n"
    << "        thresholdAngularDeg: 0.0\n"
    << "        pairingsPerPoint: 1\n"
    << "  quality:\n"
    << "    - class: mp2p_icp::QualityEvaluator_PairedRatio\n"
    << "      params: ~\n";
  return fil;
}

std::string params_yaml(bool loopClosure, const std::string & pipelineFile)
{
  std::string s = R"###(
params:
  vehicle_frame_name: "base_link"
  reference_frame_name: "map"
  kinematic_model: KinematicModel::ConstantVelocity
  max_time_to_use_velocity_model: 2.0
  sigma_random_walk_acceleration_linear: 1.0
  sigma_random_walk_acceleration_angular: 1.0
  link_first_pose_to_reference_origin_sigma: 1e-6
  min_time_difference_to_create_new_frame: 0.01
  enable_optimizer_thread: false
)###";
  if (loopClosure) {
    s += "  loop_closure_enabled: true\n";
    s += "  loop_closure_pipeline_file: \"" + pipelineFile + "\"\n";
    // Long enough that the periodic timer cannot fire during the test: what is
    // being measured is the forced scan, not whichever scan happened to be due.
    s += "  loop_closure_check_period_sec: 3600.0\n";
    s += "  loop_closure_min_new_keyframes: 1\n";
    s += "  loop_closure_finalize_rounds: 0\n";
  } else {
    s += "  loop_closure_enabled: false\n";
  }
  return s;
}

std::string diagnostic_value(mola::mapper::Mapper & m, const std::string & key)
{
  std::vector<mola::DiagnosticStatusMsg> st;
  m.getDiagnostics(st);
  for (const auto & msg : st) {
    for (const auto & kv : msg.values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
  }
  return {};
}

void insert_keyframes(mola::mapper::Mapper & m, int n)
{
  const auto t0 = mrpt::Clock::now();
  for (int i = 0; i < n; i++) {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = t0 + std::chrono::milliseconds(200 * i);
    req.source_frame_id = "odom_test";
    req.pose_in_source.mean =
      mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0 * i, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    req.pose_in_source.cov.setIdentity();
    req.pose_in_source.cov *= 0.01;
    m.requestInsertKeyframe(req);
  }
}

/// With loop closure off there is no thread, so there is nothing to wait for.
/// That must read as success: a caller should be able to wait unconditionally
/// rather than first asking whether loop closure happens to be enabled.
void test_no_thread_returns_immediately()
{
  std::cout << "  --- no LC thread ---\n";
  mola::mapper::Mapper m;
  m.setMinLoggingLevel(mrpt::system::LVL_WARN);
  m.initialize(mrpt::containers::yaml::FromText(params_yaml(false, "")));

  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = m.wait_for_loop_closure_idle(/*timeoutSeconds=*/5.0);
  const std::chrono::duration<double> dt = std::chrono::steady_clock::now() - t0;

  ASSERT_(ok);
  // "Immediately" as in: it did not sit out the timeout.
  ASSERT_LT_(dt.count(), 1.0);
  std::cout << "    returned true in " << dt.count() << " s   OK\n";
}

/// The handshake: after asking for a scan, the wait must not return until that
/// scan has actually run. Returning early -- in the window between the request
/// being consumed and the scan being marked started -- is the bug this guards.
void test_wait_covers_a_forced_scan()
{
  std::cout << "  --- forced scan, then wait ---\n";
  const auto pipelineFile = write_minimal_lc_pipeline();

  mola::mapper::Mapper m;
  m.setMinLoggingLevel(mrpt::system::LVL_WARN);
  m.initialize(mrpt::containers::yaml::FromText(params_yaml(true, pipelineFile)));

  insert_keyframes(m, 3);

  const auto scansBefore = std::stoull(diagnostic_value(m, "lc_scans"));

  // Repeat: a single pass can pass by luck if the wait returns early but the
  // scan happens to have finished anyway. Ten request/wait cycles make that
  // coincidence unlikely to hold every time.
  for (int i = 0; i < 10; i++) {
    m.request_loop_closure_scan();
    const bool ok = m.wait_for_loop_closure_idle(/*timeoutSeconds=*/60.0);
    ASSERTMSG_(ok, "wait_for_loop_closure_idle() timed out on a forced scan");

    // The whole point: on return, no scan may still be running.
    ASSERT_EQUAL_(diagnostic_value(m, "lc_scan_in_progress"), std::string("no"));
  }

  const auto scansAfter = std::stoull(diagnostic_value(m, "lc_scans"));
  std::cout << "    lc_scans " << scansBefore << " -> " << scansAfter << "\n";
  ASSERTMSG_(
    scansAfter > scansBefore,
    "the forced scans were waited on but none of them completed -- the wait is "
    "returning without the scan having run");

  mrpt::system::deleteFile(pipelineFile);
  std::cout << "    OK\n";
}

/// Waiting must not be able to hang past its own deadline. Hard to provoke
/// with a real scan (they finish in microseconds here), so this only pins that
/// the timeout path is wired and returns a bool rather than blocking forever.
void test_timeout_is_honored()
{
  std::cout << "  --- timeout path ---\n";
  const auto pipelineFile = write_minimal_lc_pipeline();

  mola::mapper::Mapper m;
  m.setMinLoggingLevel(mrpt::system::LVL_WARN);
  m.initialize(mrpt::containers::yaml::FromText(params_yaml(true, pipelineFile)));
  insert_keyframes(m, 3);

  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = m.wait_for_loop_closure_idle(/*timeoutSeconds=*/2.0);
  const std::chrono::duration<double> dt = std::chrono::steady_clock::now() - t0;

  // Idle already, so this returns true fast; the assertion that matters is that
  // it is bounded by the timeout either way.
  ASSERT_LT_(dt.count(), 3.0);
  std::cout << "    returned " << (ok ? "true" : "false") << " in " << dt.count() << " s   OK\n";

  mrpt::system::deleteFile(pipelineFile);
}
}  // namespace

int main(int, char **)
{
  try {
    test_no_thread_returns_immediately();
    test_wait_covers_a_forced_scan();
    test_timeout_is_honored();
    std::cout << "Test successful." << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
