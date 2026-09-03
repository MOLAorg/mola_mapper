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
 * @file   mola-mapper-cli.cpp
 * @brief  Offline CLI front end for Mapper: simplemap -> optimized map.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * Takes the keyframe simplemap a front end already produced (e.g.
 * `mola-lidar-odometry-cli --output-simplemap`), replays it into
 * mola::mapper::Mapper as a keyframe backbone, closes loops, optimizes, and
 * writes the corrected map back out.
 *
 * The point of running the mapper this way rather than live is REPRODUCIBILITY.
 * A live run is paced by two background threads and by wall-clock scan periods,
 * so replaying the same bag twice need not produce the same map. Here the
 * optimizer runs on this thread and every loop-closure scan is triggered after
 * a fixed number of keyframes rather than after a number of seconds. The forced
 * settings that do that are applied over the config file (see
 * force_deterministic_settings()) and cannot be overridden from YAML.
 *
 * That covers everything the mapper itself owns. It does NOT, on its own, cover
 * the loop-closure detector: that evaluates candidates on its own worker pool
 * sized from hardware_concurrency(), and underneath a single candidate mp2p_icp
 * decides both the order of its correspondence list and the summation order of
 * the Gauss-Newton normal equations with TBB reductions. Before that was
 * addressed, three identical KITTI-00 runs here gave APE 0.948 / 0.973 / 0.949 m.
 *
 * So this program also turns on the engine's own `deterministic` mode
 * (mola_sm_loop_closure), which pins those runtimes for the duration of a scan.
 * With it, a loop-closing run is bit-identical across repeats; without it, only
 * a run that closes no loop is. It costs wall clock -- one thread all the way
 * down, measured ~8x on KITTI-07 -- which is the trade this program is meant to
 * make. Pass --no-deterministic to take the fast, non-reproducible path.
 *
 * Two trajectories come out, and they answer different questions:
 *  - `--output-tum-path` is the optimized KEYFRAME trajectory: one pose per
 *    keyframe, which on a distance-gated map is far sparser than the front
 *    end's own output.
 *  - `--output-corrected-trajectory` re-applies the mapper's corrections to a
 *    DENSE front-end trajectory (`--input-trajectory`), so the result keeps the
 *    front end's sampling and local detail while picking up the global
 *    correction the graph computed. This is the one to score against a ground
 *    truth sampled at sensor rate.
 */

#include <mola_mapper/Mapper.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>

// Keep the linker from dropping the module self-registration in register.cpp.
#include <mrpt/core/initializer.h>

namespace
{
struct Cli
{
  CLI::App cmd{"mola-mapper-cli"};

  std::string argInput{"map.simplemap"};
  std::string argConfig{"mapper-offline.yaml"};
  std::string argOutputSimpleMap;
  std::string argOutputTum;
  std::string argInputTrajectory;
  std::string argOutputCorrectedTrajectory;
  std::string argLcPipeline;
  bool argNoLoopClosure{false};
  int argLcScanEvery{50};
  int argLcFinalizeRounds{-1};
  std::string argExternalsDir;
  bool argLazyLoadOutput{false};
  bool argNoDeterministic{false};
  bool argIgnoreInputTwist{false};
  double argTwistSigmaLin{0.5};
  double argTwistSigmaAng{5.0};
  std::string argVerbosity{"INFO"};

  CLI::Option * optOutputSimpleMap{nullptr};
  CLI::Option * optOutputTum{nullptr};
  CLI::Option * optInputTrajectory{nullptr};
  CLI::Option * optOutputCorrectedTrajectory{nullptr};
  CLI::Option * optLcPipeline{nullptr};

  Cli()
  {
    cmd.add_option("-i,--input", argInput, "Input .simplemap file to optimize offline.")
      ->required();
    cmd
      .add_option(
        "-c,--config", argConfig,
        "YAML parameter file for mola::mapper::Mapper. This is the same FLAT "
        "fragment the mola-cli launchers `$import` (e.g. the installed "
        "share/mola_mapper/params/mapper-offline.yaml), not a whole mola-cli "
        "system definition.")
      ->required();
    optOutputSimpleMap = cmd.add_option(
      "-o,--output-simplemap", argOutputSimpleMap,
      "Write the optimized map here (keyframe poses corrected, observations "
      "preserved).");
    optOutputTum = cmd.add_option(
      "--output-tum-path", argOutputTum,
      "Write the optimized KEYFRAME trajectory here, in TUM format.");
    optInputTrajectory = cmd.add_option(
      "--input-trajectory", argInputTrajectory,
      "Dense front-end trajectory (TUM) to correct with the mapper's optimized "
      "keyframe poses. Requires --output-corrected-trajectory.");
    optOutputCorrectedTrajectory = cmd.add_option(
      "--output-corrected-trajectory", argOutputCorrectedTrajectory,
      "Write the corrected DENSE trajectory here, in TUM format. Requires "
      "--input-trajectory.");
    optLcPipeline = cmd.add_option(
      "--lc-pipeline", argLcPipeline,
      "Override the loop-closure pipeline YAML (mola_sm_loop_closure F2F "
      "pipeline) named by the config file.");
    cmd.add_flag(
      "--no-loop-closure", argNoLoopClosure,
      "Disable loop closure: optimize the odometry chain alone. Useful as the "
      "reference arm for measuring what loop closure buys.");
    cmd.add_option(
      "--lc-scan-every", argLcScanEvery,
      "Run one loop-closure scan every N ingested keyframes (0 = none during "
      "ingestion, rely on the finalize pass alone). Default: 50.");
    cmd.add_option(
      "--lc-finalize-rounds", argLcFinalizeRounds,
      "Override the config's loop_closure_finalize_rounds: full scan + "
      "re-optimization rounds after ingestion, until a round finds nothing new.");
    cmd.add_option(
      "--externals-dir", argExternalsDir,
      "Lazy-load base directory for a simplemap with externally-stored "
      "observations. If unset, a sibling directory named '<input>_Images' is "
      "used when it exists.");
    cmd.add_flag(
      "--lazy-load-output", argLazyLoadOutput,
      "Externalize the output simplemap's point clouds into a sidecar directory, "
      "so it stays small and loads fast downstream.");
    cmd.add_flag(
      "--no-deterministic", argNoDeterministic,
      "Let the loop-closure detector use all cores. Much faster, and the run is "
      "then NOT reproducible: two runs that close loops give different maps. Use "
      "when the map matters and repeating the exact run does not.");
    cmd.add_flag(
      "--ignore-input-twist", argIgnoreInputTwist,
      "Do not feed the input simplemap's per-keyframe velocities into the graph. "
      "Diagnostic only: without them the constant-velocity factor between "
      "keyframes has no velocity observation to agree with and deforms the map "
      "(measured 7.1 -> 34.1 m APE on KITTI-00).");
    cmd.add_option(
      "--twist-sigma-lin", argTwistSigmaLin, "Sigma [m/s] of the input keyframe velocities.");
    cmd.add_option(
      "--twist-sigma-ang", argTwistSigmaAng,
      "Sigma [deg/s] of the input keyframe angular velocities.");
    cmd.add_option(
      "-v,--verbosity", argVerbosity, "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");
  }
};

/// The timestamp a keyframe stands for. A simplemap stores no keyframe stamp of
/// its own, so it has to come from the observations, and it must be the SAME
/// instant the front end used when it emitted its dense pose: otherwise the
/// correction field built below would be sampled off by one scan period.
/// Front ends stamp every observation of a keyframe at its own scan time, so
/// the earliest one is that instant.
std::optional<mrpt::Clock::time_point> keyframe_timestamp(const mrpt::obs::CSensoryFrame & sf)
{
  std::optional<mrpt::Clock::time_point> best;
  for (const auto & obs : sf) {
    if (!obs) {
      continue;
    }
    if (!best.has_value() || obs->timestamp < *best) {
      best = obs->timestamp;
    }
  }
  return best;
}

/// Overrides that make an offline run a function of its input alone. Applied
/// AFTER the user's config file, so no YAML can silently reintroduce a
/// wall-clock-paced or racing code path into a run whose whole purpose is to be
/// reproducible.
void force_deterministic_settings(mrpt::containers::yaml & params)
{
  // Both background threads off: the solve and every loop-closure scan then
  // happen where this program asks for them, in a fixed order.
  params["enable_optimizer_thread"] = false;
  params["enable_loop_closure_thread"] = false;
  // Nothing subscribes to a high-rate pose here, and its cadence is wall-clock.
  params["high_rate_pose_publish_rate_hz"] = 0.0;
  // The keyframe backbone is exactly what this program feeds in; no sensor path
  // may add a keyframe variable of its own.
  params["keyframe_creation_source"] = "KeyframeCreationSource::SharedMapOnly";
  // Saving is done explicitly below, after finalize, so the destructor's own
  // save cannot race or double-write.
  params["save_simplemap_file"] = "";
  params["save_trajectory_to_file"] = "";
}

/// Poses are corrected in the world frame: `corrected = C(t) o original`, where
/// C is interpolated between the per-keyframe corrections
/// `C_i = optimized_i o original_i^-1`. Composing on the left is what makes this
/// a global deformation: it moves the trajectory into the optimized frame while
/// leaving the front end's local geometry between keyframes intact, which is
/// the part the mapper's sparse keyframes cannot represent and the part a
/// sensor-rate metric actually measures.
mrpt::poses::CPose3DInterpolator build_correction_field(
  const std::map<mrpt::Clock::time_point, mrpt::poses::CPose3D> & originalKfPoses,
  const mrpt::poses::CPose3DInterpolator & optimizedKfPoses)
{
  mrpt::poses::CPose3DInterpolator corr;
  corr.setInterpolationMethod(mrpt::poses::imLinearSlerp);

  for (const auto & [t, optimized] : optimizedKfPoses) {
    const auto it = originalKfPoses.find(t);
    if (it == originalKfPoses.end()) {
      continue;
    }
    corr.insert(t, mrpt::poses::CPose3D(optimized) + (-it->second));
  }
  return corr;
}

std::size_t write_corrected_trajectory(
  const std::string & inFile, const std::string & outFile,
  const mrpt::poses::CPose3DInterpolator & corr)
{
  ASSERT_(!corr.empty());

  mrpt::poses::CPose3DInterpolator dense;
  if (!dense.loadFromTextFile_TUM(inFile)) {
    THROW_EXCEPTION_FMT("Could not read input trajectory in TUM format: '%s'", inFile.c_str());
  }

  // Outside the keyframes' time span there is nothing to interpolate between,
  // so the nearest correction is held. That keeps the samples before the first
  // and after the last keyframe (a front end typically emits a few of each)
  // instead of dropping them, which a metric would otherwise read as coverage
  // loss rather than as the boundary condition it is.
  const auto firstCorr = corr.begin()->second;
  const auto lastCorr = corr.rbegin()->second;
  const auto tFirst = corr.begin()->first;
  const auto tLast = corr.rbegin()->first;

  mrpt::poses::CPose3DInterpolator out;
  for (const auto & [t, pose] : dense) {
    mrpt::math::TPose3D C;
    if (t <= tFirst) {
      C = firstCorr;
    } else if (t >= tLast) {
      C = lastCorr;
    } else {
      bool valid = false;
      corr.interpolate(t, C, valid);
      if (!valid) {
        continue;
      }
    }
    out.insert(t, mrpt::poses::CPose3D(C) + mrpt::poses::CPose3D(pose));
  }

  if (!out.saveToTextFile_TUM(outFile)) {
    THROW_EXCEPTION_FMT("Could not write corrected trajectory: '%s'", outFile.c_str());
  }
  return out.size();
}

void run_offline_mapping(Cli & cli)
{
  // ---------------------------------------------------------------------
  // 1) Configuration
  // ---------------------------------------------------------------------
  const auto configFile = cli.argConfig;
  ASSERT_FILE_EXISTS_(configFile);

  mrpt::containers::yaml params = mola::load_yaml_file(configFile);
  ASSERTMSG_(
    params.isMap(),
    "The config file must be a map of mola::mapper::Mapper parameters (the "
    "same flat fragment the launchers `$import`).");

  if (cli.optLcPipeline->count() > 0) {
    params["loop_closure_pipeline_file"] = cli.argLcPipeline;
  }
  if (cli.argNoLoopClosure) {
    params["loop_closure_enabled"] = false;
  }
  if (cli.argLcFinalizeRounds >= 0) {
    params["loop_closure_finalize_rounds"] = cli.argLcFinalizeRounds;
  }
  if (cli.argLazyLoadOutput) {
    params["generate_lazy_load_scan_files"] = true;
  }

  force_deterministic_settings(params);

  // The loop-closure engine's own reproducibility switch. It lives in the LC
  // PIPELINE yaml, not in the mapper's params, and that file is shared with the
  // online launchers -- where this must stay off, since it is one thread all the
  // way down and those runs are paced by a real-time clock. So it is bound here,
  // per-process, via the ${LC_DETERMINISTIC} hook the pipeline already exposes,
  // rather than by editing the shared file.
  //
  // Set BEFORE initialize(): mola_yaml resolves those hooks when the engine
  // loads the pipeline, which happens inside Mapper::initialize().
  //
  // On by default because that is what this program is for. An explicitly
  // exported LC_DETERMINISTIC still wins over the default (overwrite=0), while
  // --no-deterministic wins over both.
  if (cli.argNoDeterministic) {
    ::setenv("LC_DETERMINISTIC", "false", /*overwrite=*/1);
    std::cout << "[mola-mapper-cli] Loop closure: parallel (NOT reproducible).\n";
  } else {
    ::setenv("LC_DETERMINISTIC", "true", /*overwrite=*/0);
  }

  mrpt::containers::yaml cfg = mrpt::containers::yaml::Map();
  cfg["params"] = params;

  // ---------------------------------------------------------------------
  // 2) Input map
  // ---------------------------------------------------------------------
  const auto inputFile = cli.argInput;
  ASSERT_FILE_EXISTS_(inputFile);

  std::string lazyLoadBaseDir = cli.argExternalsDir;
  if (lazyLoadBaseDir.empty()) {
    const auto candidate = mrpt::system::pathJoin(
      {mrpt::system::extractFileDirectory(inputFile),
       mrpt::system::extractFileName(inputFile) + "_Images"});
    if (mrpt::system::directoryExists(candidate)) {
      lazyLoadBaseDir = candidate;
    }
  }
  if (!lazyLoadBaseDir.empty()) {
    std::cout << "[mola-mapper-cli] Lazy-load base directory: '" << lazyLoadBaseDir << "'\n";
    mrpt::io::setLazyLoadPathBase(lazyLoadBaseDir);
  }

  std::cout << "[mola-mapper-cli] Reading simplemap from: '" << inputFile << "'...\n";
  mrpt::maps::CSimpleMap sm;
  if (!sm.loadFromFile(inputFile)) {
    THROW_EXCEPTION_FMT("Could not read input simplemap: '%s'", inputFile.c_str());
  }
  ASSERTMSG_(!sm.empty(), "The input simplemap has no keyframes.");
  std::cout << "[mola-mapper-cli] Read " << sm.size() << " keyframes.\n";

  // ---------------------------------------------------------------------
  // 3) Mapper
  // ---------------------------------------------------------------------
  auto mapper = mola::mapper::Mapper::Create();
  ASSERT_(mapper);

  {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    mapper->setMinLoggingLevel(vl::name2value(cli.argVerbosity));
  }
  mapper->initialize(cfg);

  // ---------------------------------------------------------------------
  // 4) Replay the keyframe backbone, in time order
  // ---------------------------------------------------------------------
  // A simplemap is stored in insertion order, which is normally chronological,
  // but the mapper snaps an out-of-order request to the nearest existing
  // keyframe rather than inserting a variable in the past. Sorting here keeps a
  // map whose order was disturbed (e.g. by a merge) from silently collapsing
  // keyframes together.
  struct InputKeyframe
  {
    mrpt::Clock::time_point stamp;
    mrpt::poses::CPose3DPDFGaussian pose;
    mrpt::obs::CSensoryFrame::Ptr sf;
    std::optional<mrpt::math::TTwist3D> twist;
  };
  std::vector<InputKeyframe> keyframes;
  keyframes.reserve(sm.size());

  std::size_t skippedNoStamp = 0;
  for (const auto & kf : sm) {
    if (!kf.sf || !kf.pose) {
      skippedNoStamp++;
      continue;
    }
    const auto stamp = keyframe_timestamp(*kf.sf);
    if (!stamp.has_value()) {
      skippedNoStamp++;
      continue;
    }
    InputKeyframe ikf;
    ikf.stamp = *stamp;
    ikf.pose.copyFrom(*kf.pose);
    ikf.sf = kf.sf;
    ikf.twist = kf.localTwist;
    keyframes.push_back(std::move(ikf));
  }
  if (skippedNoStamp != 0) {
    std::cout << "[mola-mapper-cli] WARNING: skipped " << skippedNoStamp
              << " keyframe(s) with no observations to take a timestamp from.\n";
  }
  ASSERTMSG_(!keyframes.empty(), "No usable keyframes in the input simplemap.");

  std::stable_sort(keyframes.begin(), keyframes.end(), [](const auto & a, const auto & b) {
    return a.stamp < b.stamp;
  });

  // Keep each keyframe's INPUT pose: the correction field below is the
  // difference between these and the optimized ones.
  std::map<mrpt::Clock::time_point, mrpt::poses::CPose3D> originalKfPoses;

  const int scanEvery = std::max(0, cli.argLcScanEvery);
  std::size_t loopsOnline = 0;

  // Per-keyframe velocity, and why it is not optional in practice. The graph
  // carries a body-frame velocity variable per keyframe, coupled to its
  // neighbors by a constant-velocity factor. Live, the front end pins those
  // variables by calling fuse_pose()/fuse_twist() densely between keyframes.
  // Replaying a simplemap there is no such dense source, so with no velocity
  // observation the constant-velocity factor stops being a motion model and
  // becomes a smoothness prior that fights the relative-pose chain wherever
  // the platform actually accelerates -- 7.1 -> 34.1 m APE on KITTI-00, a
  // corner-cutting deformation, not a divergence. A simplemap already stores
  // the front end's own per-keyframe twist, so feed it.
  const bool useTwist = !cli.argIgnoreInputTwist;
  mrpt::math::CMatrixDouble66 twistCov;
  twistCov.setZero();
  {
    const double sl = cli.argTwistSigmaLin;
    const double sa = mrpt::DEG2RAD(cli.argTwistSigmaAng);
    ASSERTMSG_(sl > 0 && sa > 0, "--twist-sigma-lin/ang must be positive.");
    for (int i = 0; i < 3; i++) {
      twistCov(i, i) = sl * sl;
      twistCov(i + 3, i + 3) = sa * sa;
    }
  }
  std::size_t twistFed = 0;

  for (std::size_t i = 0; i < keyframes.size(); i++) {
    const auto & ikf = keyframes[i];

    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = ikf.stamp;
    // A dedicated source name, distinct from any dense `fuse_pose()` frame, per
    // the SharedKeyframeMap contract.
    req.source_frame_id = "offline_odom_kf";
    req.pose_in_source = ikf.pose;
    req.observations = *ikf.sf;
    mapper->requestInsertKeyframe(req);

    // AFTER the insert, and at the same timestamp, so it lands on the keyframe
    // just created rather than creating one of its own.
    if (useTwist && ikf.twist.has_value()) {
      mapper->fuse_twist(ikf.stamp, *ikf.twist, twistCov);
      twistFed++;
    }

    originalKfPoses[ikf.stamp] = ikf.pose.mean;

    if (scanEvery > 0 && ((i + 1) % static_cast<std::size_t>(scanEvery)) == 0) {
      // Solve first: the detector proposes candidates from the CURRENT poses,
      // so scanning on a stale (un-optimized) map wastes the scan's budget on
      // pairs that are no longer geometrically plausible.
      mapper->optimize_now();
      const auto merged = mapper->run_loop_closure_scan_now(/*forceFullScan=*/false);
      loopsOnline += merged;
      std::cout << "[mola-mapper-cli] keyframe " << (i + 1) << "/" << keyframes.size()
                << ": loop-closure scan merged " << merged << " edge(s), " << loopsOnline
                << " so far.\n";
    }
  }

  std::cout << "[mola-mapper-cli] Ingested " << keyframes.size() << " keyframes ("
            << mapper->keyframe_count() << " in the central map), " << twistFed
            << " with a velocity.\n";
  if (useTwist && twistFed == 0) {
    std::cout << "[mola-mapper-cli] WARNING: no keyframe in this simplemap carries a "
                 "velocity, so the graph's constant-velocity factors have nothing to "
                 "agree with and may deform the map. Check how the map was written.\n";
  }

  // ---------------------------------------------------------------------
  // 5) Close what is left, then solve
  // ---------------------------------------------------------------------
  mapper->optimize_now();
  mapper->finalize_loop_closures();
  mapper->optimize_now();

  // ---------------------------------------------------------------------
  // 6) Outputs
  // ---------------------------------------------------------------------
  const auto optimizedKfPoses = mapper->estimated_keyframe_trajectory();
  std::cout << "[mola-mapper-cli] Optimized " << optimizedKfPoses.size() << " keyframe poses.\n";
  ASSERTMSG_(!optimizedKfPoses.empty(), "The optimizer committed no keyframe poses.");

  if (cli.optOutputTum->count() > 0 && !cli.argOutputTum.empty()) {
    const auto & fil = cli.argOutputTum;
    if (!optimizedKfPoses.saveToTextFile_TUM(fil)) {
      THROW_EXCEPTION_FMT("Could not write keyframe trajectory: '%s'", fil.c_str());
    }
    std::cout << "[mola-mapper-cli] Wrote keyframe trajectory: '" << fil << "' ("
              << optimizedKfPoses.size() << " poses).\n";
  }

  if (cli.optOutputSimpleMap->count() > 0 && !cli.argOutputSimpleMap.empty()) {
    const auto & fil = cli.argOutputSimpleMap;
    const auto outMap = mapper->current_simple_map();
    if (!outMap.saveToFile(fil)) {
      THROW_EXCEPTION_FMT("Could not write output simplemap: '%s'", fil.c_str());
    }
    std::cout << "[mola-mapper-cli] Wrote optimized simplemap: '" << fil << "' (" << outMap.size()
              << " keyframes).\n";
  }

  const bool wantCorrected =
    cli.optInputTrajectory->count() > 0 || cli.optOutputCorrectedTrajectory->count() > 0;
  if (wantCorrected) {
    ASSERTMSG_(
      cli.optInputTrajectory->count() > 0 && cli.optOutputCorrectedTrajectory->count() > 0,
      "--input-trajectory and --output-corrected-trajectory must be used together.");

    const auto corr = build_correction_field(originalKfPoses, optimizedKfPoses);
    ASSERTMSG_(
      !corr.empty(),
      "No keyframe timestamp survived into the optimized trajectory, so no "
      "correction field could be built.");

    const auto n =
      write_corrected_trajectory(cli.argInputTrajectory, cli.argOutputCorrectedTrajectory, corr);
    std::cout << "[mola-mapper-cli] Wrote corrected dense trajectory: '"
              << cli.argOutputCorrectedTrajectory << "' (" << n << " poses, corrected "
              << "from " << corr.size() << " keyframe corrections).\n";
  }

  std::cout << "[mola-mapper-cli] Done.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    CLI11_PARSE(cli.cmd, argc, argv);
    run_offline_mapping(cli);
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
