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
 * @file   mola-mapper-reoptimize.cpp
 * @brief  Re-optimizes a global graph saved by the mapper, with other parameters.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GlobalGraph.h>
#include <mola_mapper/GlobalGraphIO.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void usage()
{
  std::cerr << "Usage: mola-mapper-reoptimize INPUT_PREFIX MAPPER_YAML OUTPUT_PREFIX [options]\n"
               "  INPUT_PREFIX:  files saved by the mapper with output_graph_input\n"
               "  MAPPER_YAML:   mapper parameters (its params.graph section is used; environment\n"
               "                 variables apply, as in a launch)\n"
               "  OUTPUT_PREFIX: writes OUTPUT_PREFIX.tum and OUTPUT_PREFIX_georef.yaml\n"
               "Options:\n"
               "  --no-loops         ignore the loop closures\n"
               "  --keep-initial     start from the saved initial guesses (default: chain the\n"
               "                     front end increments from the first keyframe)\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 4) {
      usage();
      return 1;
    }
    const std::string inPrefix = argv[1];
    const std::string paramsFile = argv[2];
    const std::string outPrefix = argv[3];
    bool noLoops = false;
    bool keepInitial = false;
    for (int i = 4; i < argc; i++) {
      const std::string a = argv[i];
      if (a == "--no-loops") {
        noLoops = true;
      } else if (a == "--keep-initial") {
        keepInitial = true;
      } else {
        usage();
        return 1;
      }
    }

    const auto cfg = mola::load_yaml_file(paramsFile);
    mola::mapper::GlobalGraphParams params;
    if (cfg.has("params") && cfg["params"].has("graph")) {
      params.loadFrom(cfg["params"]["graph"]);
    }

    auto in = mola::mapper::load_global_graph_input(inPrefix);
    if (noLoops) {
      in.loops.clear();
    }
    if (!keepInitial) {
      // Chained within each front end source frame. Across a change of source
      // frame, the saved relative pose is kept, since it defines the weak edge.
      std::vector<mrpt::poses::CPose3D> saved;
      for (const auto & kf : in.keyframes) {
        saved.push_back(kf.initial_guess);
      }
      for (std::size_t i = 1; i < in.keyframes.size(); i++) {
        auto & kf = in.keyframes[i];
        const auto & prev = in.keyframes[i - 1];
        if (kf.source_frame == prev.source_frame) {
          kf.initial_guess =
            prev.initial_guess + (kf.pose_in_source.mean - prev.pose_in_source.mean);
        } else {
          kf.initial_guess = prev.initial_guess + (saved[i] - saved[i - 1]);
        }
      }
    }

    mrpt::system::COutputLogger logger("reoptimize");
    logger.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    const auto r = mola::mapper::optimize_global_graph(in, params, &logger);

    std::vector<std::pair<mrpt::Clock::time_point, mrpt::poses::CPose3D>> poses;
    for (std::size_t i = 0; i < r.poses.size(); i++) {
      poses.emplace_back(in.keyframes[i].stamp, r.poses[i]);
    }
    mola::mapper::save_tum_trajectory(poses, outPrefix + ".tum");
    if (r.georeferencing) {
      mola::mapper::save_georeference(*r.georeferencing, outPrefix + "_georef.yaml");
    } else {
      // Not one of an earlier run with the same prefix:
      std::remove((outPrefix + "_georef.yaml").c_str());
    }
    std::cout << "keyframes: " << r.poses.size() << ", odometry: " << r.num_odometry_factors
              << ", loops: " << r.num_loop_factors << ", GNSS: " << r.num_gnss_factors
              << (r.gnss_degenerate ? " (degenerate)" : "")
              << ", gravity: " << r.num_gravity_factors << ", error " << r.initial_error << " -> "
              << r.final_error << "\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
