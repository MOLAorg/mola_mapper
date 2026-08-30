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
 * optimizer and the loop-closure engine both run on this thread, and every
 * loop-closure scan is triggered after a fixed number of keyframes rather than
 * after a number of seconds, so the output is a function of the input alone.
 * The forced settings that guarantee that are applied over the config file
 * (see force_deterministic_settings()) and cannot be overridden from YAML.
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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
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
  TCLAP::CmdLine cmd{"mola-mapper-cli", ' ', "0.1.0"};

  TCLAP::ValueArg<std::string> argInput{
    "i",  "input",         "Input .simplemap file to optimize offline.",
    true, "map.simplemap", "map.simplemap",
    cmd};

  TCLAP::ValueArg<std::string> argConfig{
    "c",
    "config",
    "YAML parameter file for mola::mapper::Mapper. This is the same FLAT "
    "fragment the mola-cli launchers `$import` (e.g. the installed "
    "share/mola_mapper/params/mapper-offline.yaml), not a whole mola-cli "
    "system definition.",
    true,
    "mapper-offline.yaml",
    "mapper-offline.yaml",
    cmd};

  TCLAP::ValueArg<std::string> argOutputSimpleMap{
    "o",
    "output-simplemap",
    "Write the optimized map here (keyframe poses corrected, observations "
    "preserved).",
    false,
    "",
    "optimized.simplemap",
    cmd};

  TCLAP::ValueArg<std::string> argOutputTum{
    "",
    "output-tum-path",
    "Write the optimized KEYFRAME trajectory here, in TUM format.",
    false,
    "",
    "keyframes.tum",
    cmd};

  TCLAP::ValueArg<std::string> argInputTrajectory{
    "",
    "input-trajectory",
    "Dense front-end trajectory (TUM) to correct with the mapper's optimized "
    "keyframe poses. Requires --output-corrected-trajectory.",
    false,
    "",
    "lidar_odom.tum",
    cmd};

  TCLAP::ValueArg<std::string> argOutputCorrectedTrajectory{
    "",
    "output-corrected-trajectory",
    "Write the corrected DENSE trajectory here, in TUM format. Requires "
    "--input-trajectory.",
    false,
    "",
    "corrected.tum",
    cmd};

  TCLAP::ValueArg<std::string> argLcPipeline{
    "",
    "lc-pipeline",
    "Override the loop-closure pipeline YAML (mola_sm_loop_closure F2F "
    "pipeline) named by the config file.",
    false,
    "",
    "loop-closure-f2f-mapper.yaml",
    cmd};

  TCLAP::SwitchArg argNoLoopClosure{
    "", "no-loop-closure",
    "Disable loop closure: optimize the odometry chain alone. Useful as the "
    "reference arm for measuring what loop closure buys.",
    cmd};

  TCLAP::ValueArg<int> argLcScanEvery{
    "",
    "lc-scan-every",
    "Run one loop-closure scan every N ingested keyframes (0 = none during "
    "ingestion, rely on the finalize pass alone). Default: 50.",
    false,
    50,
    "N",
    cmd};

  TCLAP::ValueArg<int> argLcFinalizeRounds{
    "",
    "lc-finalize-rounds",
    "Override the config's loop_closure_finalize_rounds: full scan + "
    "re-optimization rounds after ingestion, until a round finds nothing new.",
    false,
    -1,
    "N",
    cmd};

  TCLAP::ValueArg<std::string> argExternalsDir{
    "",
    "externals-dir",
    "Lazy-load base directory for a simplemap with externally-stored "
    "observations. If unset, a sibling directory named '<input>_Images' is "
    "used when it exists.",
    false,
    "",
    "<ExternalsDirectory>",
    cmd};

  TCLAP::SwitchArg argLazyLoadOutput{
    "", "lazy-load-output",
    "Externalize the output simplemap's point clouds into a sidecar directory, "
    "so it stays small and loads fast downstream.",
    cmd};

  TCLAP::ValueArg<std::string> argVerbosity{
    "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "INFO",
    "INFO", cmd};
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
  const auto configFile = cli.argConfig.getValue();
  ASSERT_FILE_EXISTS_(configFile);

  mrpt::containers::yaml params = mola::load_yaml_file(configFile);
  ASSERTMSG_(
    params.isMap(),
    "The config file must be a map of mola::mapper::Mapper parameters (the "
    "same flat fragment the launchers `$import`).");

  if (cli.argLcPipeline.isSet()) {
    params["loop_closure_pipeline_file"] = cli.argLcPipeline.getValue();
  }
  if (cli.argNoLoopClosure.getValue()) {
    params["loop_closure_enabled"] = false;
  }
  if (cli.argLcFinalizeRounds.getValue() >= 0) {
    params["loop_closure_finalize_rounds"] = cli.argLcFinalizeRounds.getValue();
  }
  if (cli.argLazyLoadOutput.getValue()) {
    params["generate_lazy_load_scan_files"] = true;
  }

  force_deterministic_settings(params);

  mrpt::containers::yaml cfg = mrpt::containers::yaml::Map();
  cfg["params"] = params;

  // ---------------------------------------------------------------------
  // 2) Input map
  // ---------------------------------------------------------------------
  const auto inputFile = cli.argInput.getValue();
  ASSERT_FILE_EXISTS_(inputFile);

  std::string lazyLoadBaseDir = cli.argExternalsDir.getValue();
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
    mapper->setMinLoggingLevel(vl::name2value(cli.argVerbosity.getValue()));
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

  const int scanEvery = std::max(0, cli.argLcScanEvery.getValue());
  std::size_t loopsOnline = 0;

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
            << mapper->keyframe_count() << " in the central map).\n";

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

  if (cli.argOutputTum.isSet() && !cli.argOutputTum.getValue().empty()) {
    const auto & fil = cli.argOutputTum.getValue();
    if (!optimizedKfPoses.saveToTextFile_TUM(fil)) {
      THROW_EXCEPTION_FMT("Could not write keyframe trajectory: '%s'", fil.c_str());
    }
    std::cout << "[mola-mapper-cli] Wrote keyframe trajectory: '" << fil << "' ("
              << optimizedKfPoses.size() << " poses).\n";
  }

  if (cli.argOutputSimpleMap.isSet() && !cli.argOutputSimpleMap.getValue().empty()) {
    const auto & fil = cli.argOutputSimpleMap.getValue();
    const auto outMap = mapper->current_simple_map();
    if (!outMap.saveToFile(fil)) {
      THROW_EXCEPTION_FMT("Could not write output simplemap: '%s'", fil.c_str());
    }
    std::cout << "[mola-mapper-cli] Wrote optimized simplemap: '" << fil << "' (" << outMap.size()
              << " keyframes).\n";
  }

  const bool wantCorrected =
    cli.argInputTrajectory.isSet() || cli.argOutputCorrectedTrajectory.isSet();
  if (wantCorrected) {
    ASSERTMSG_(
      cli.argInputTrajectory.isSet() && cli.argOutputCorrectedTrajectory.isSet(),
      "--input-trajectory and --output-corrected-trajectory must be used together.");

    const auto corr = build_correction_field(originalKfPoses, optimizedKfPoses);
    ASSERTMSG_(
      !corr.empty(),
      "No keyframe timestamp survived into the optimized trajectory, so no "
      "correction field could be built.");

    const auto n = write_corrected_trajectory(
      cli.argInputTrajectory.getValue(), cli.argOutputCorrectedTrajectory.getValue(), corr);
    std::cout << "[mola-mapper-cli] Wrote corrected dense trajectory: '"
              << cli.argOutputCorrectedTrajectory.getValue() << "' (" << n << " poses, corrected "
              << "from " << corr.size() << " keyframe corrections).\n";
  }

  std::cout << "[mola-mapper-cli] Done.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    if (!cli.cmd.parse(argc, argv)) {
      return 1;
    }
    run_offline_mapping(cli);
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
