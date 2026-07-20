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
 * @brief  Offline CLI front end for Mapper (skeleton).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>

#include <iostream>

// Keep the linker from dropping the module self-registration in register.cpp.
#include <mrpt/core/initializer.h>

int main(int argc, char ** argv)
{
  try {
    TCLAP::CmdLine cmd("mola-mapper-cli", ' ', "0.1.0");

    TCLAP::ValueArg<std::string> argInput(
      "i", "input", "Input dataset (.simplemap / rosbag2) to process offline.", false, "",
      "map.simplemap", cmd);
    TCLAP::ValueArg<std::string> argConfig(
      "c", "config", "YAML configuration file for Mapper.", false, "", "mapper.yaml", cmd);

    if (!cmd.parse(argc, argv)) {
      return 1;
    }

    // Touch the type so the module is linked in:
    auto m = mola::mapper::Mapper::Create();
    ASSERT_(m);

    std::cout << "[mola-mapper-cli] Offline processing is not implemented yet "
                 "(Mapper is under construction).\n";

    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
