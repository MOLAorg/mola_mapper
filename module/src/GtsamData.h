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
 * @file   GtsamData.h
 * @brief  Definition of WorldModelState::GtsamData (private, GTSAM-visible).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 *
 * This private header defines the GTSAM pimpl payload. It is shared by the
 * translation units that need to touch the factor graph (WorldModelState.cpp,
 * Mapper3D_Fusion.cpp) while keeping GTSAM out of the public Mapper3D headers.
 */
#pragma once

#include <mola_mapper_3d/WorldModelState.h>

// GTSAM:
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>

namespace mola::mapper_3d
{
struct WorldModelState::GtsamData
{
  GtsamData() = default;

  /// Incremental nonlinear solver over ALL keyframes (the central map; not a
  /// fixed-lag window). Empty until reinitialize_gtsam_locked() configures it.
  std::optional<gtsam::ISAM2> isam2;

  /// Factors/values pending the next isam2 update.
  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newValues;

  /// Latest full optimized estimate (mirror of isam2->calculateEstimate()).
  gtsam::Values estimate;
};

}  // namespace mola::mapper_3d
