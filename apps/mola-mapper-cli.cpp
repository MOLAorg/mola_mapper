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
 * @brief  Replays a recorded mapper input offline, reproducibly.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mola_mapper/MapperReplay.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>
#include <tbb/global_control.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

namespace
{
void usage()
{
  std::cerr
    << "Usage: mola-mapper-cli INPUT_LOG MAPPER_YAML OUTPUT_DIR [options]\n"
       "  INPUT_LOG:   the mapper input recorded with record_input (MOLA_MAPPER_RECORD_INPUT)\n"
       "  MAPPER_YAML: mapper parameters (environment variables apply, as in a launch)\n"
       "  OUTPUT_DIR:  writes mapper.tum, mapper_georef.yaml and loops.txt there\n"
       "Options:\n"
       "  --lc-period S     data time between loop closure scans [s]\n"
       "                    (default: loop_closure_check_period_sec)\n"
       "  --solve-period S  data time between solves [s] (default: optimizer_min_period)\n"
       "  --save-simplemap  also writes mapper.simplemap\n"
       "  --save-graph      also writes the graph input (graph.*), for mola-mapper-reoptimize\n"
       "  --threads N       threads of the parallel algorithms (default: 1). With more,\n"
       "                    runs are faster but not bit-exact (parallel sums), and even the\n"
       "                    accepted loops may differ\n"
       "  --quiet           only warnings and the summary\n"
       "The background threads of the mapper are disabled: loop closure scans and solves\n"
       "run at fixed periods of data time, so the result does not depend on the computer.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 4) {
      usage();
      return 1;
    }
    const std::string inputLog = argv[1];
    const std::string paramsFile = argv[2];
    const std::string outDir = argv[3];
    mola::mapper::ReplayOptions opts;
    bool saveSimplemap = false;
    bool saveGraph = false;
    bool quiet = false;
    int threads = 1;
    for (int i = 4; i < argc; i++) {
      const std::string a = argv[i];
      if (a == "--lc-period" && i + 1 < argc) {
        opts.loop_closure_period = std::stod(argv[++i]);
      } else if (a == "--solve-period" && i + 1 < argc) {
        opts.solve_period = std::stod(argv[++i]);
      } else if (a == "--save-simplemap") {
        saveSimplemap = true;
      } else if (a == "--save-graph") {
        saveGraph = true;
      } else if (a == "--threads" && i + 1 < argc) {
        threads = std::max(1, std::stoi(argv[++i]));
      } else if (a == "--quiet") {
        quiet = true;
      } else {
        usage();
        return 1;
      }
    }
    ASSERT_FILE_EXISTS_(inputLog);
    const tbb::global_control tbbThreads(
      tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(threads));
    if (!mrpt::system::directoryExists(outDir)) {
      ASSERTMSG_(mrpt::system::createDirectory(outDir), "Cannot create " + outDir);
    }
    const auto out = [&](const std::string & name) {
      return mrpt::system::pathJoin({outDir, name});
    };

    auto cfg = mola::load_yaml_file(paramsFile);
    ASSERTMSG_(cfg.has("params"), "The mapper YAML must have a `params` entry.");
    auto p = cfg["params"];
    p["enable_optimizer_thread"] = false;
    p["enable_loop_closure_thread"] = false;
    p["feedback_to_state_estimator"] = false;
    p["record_input"] = std::string();
    p["output_tum_trajectory"] = out("mapper.tum");
    p["output_georeference"] = out("mapper_georef.yaml");
    p["output_loop_closures"] = out("loops.txt");
    p["output_simplemap"] = saveSimplemap ? out("mapper.simplemap") : std::string();
    p["output_graph_input"] = saveGraph ? out("graph") : std::string();
    p["output_state_estimator_tum"] = std::string();
    p["output_state_estimator_georeference"] = std::string();
    cfg["params"] = p;

    mola::mapper::Mapper mapper;
    mapper.setMinLoggingLevel(quiet ? mrpt::system::LVL_WARN : mrpt::system::LVL_INFO);
    mapper.initialize(cfg);

    const auto t0 = std::chrono::steady_clock::now();
    const auto stats = mola::mapper::replay_mapper_input(mapper, inputLog, opts);
    mapper.finalize();
    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::cout << "Replayed " << stats.keyframes << " keyframes and " << stats.observations
              << " observations (" << stats.data_duration << " s of data) in " << dt
              << " s: " << stats.loop_closure_scans << " loop closure scans, " << stats.solves
              << " solves, " << mapper.loop_closure_count() << " loops. Outputs in " << outDir
              << "\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
