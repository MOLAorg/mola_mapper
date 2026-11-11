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
 * @file   GlobalGraph.h
 * @brief  Batch pose graph over all the keyframes of the central map.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mola_mapper/Parameters.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/topography/data_types.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace mola::mapper
{
/** One keyframe of the global graph. */
struct GlobalGraphKeyframe
{
  mrpt::Clock::time_point stamp;

  /// Odometry frame of pose_in_source (e.g. "lidar_odom"): consecutive keyframes
  /// of the same source are linked by their relative pose.
  std::string source_frame;

  /// Pose (and covariance) of the vehicle in source_frame, from the front end.
  mrpt::poses::CPose3DPDFGaussian pose_in_source;

  /// Initial guess of the vehicle pose in the map frame.
  mrpt::poses::CPose3D initial_guess;

  /// GNSS readings (CObservationGPS) and gravity (averaged CObservationIMU)
  /// attached to this keyframe. May be null.
  mrpt::obs::CSensoryFrame::ConstPtr aux_observations;

  /// Rotation of the vehicle since the previous keyframe, from a gyroscope (only
  /// the rotation part is used), and its sigma per axis [rad].
  std::optional<mrpt::poses::CPose3D> gyro_rotation_from_prev;
  double gyro_rotation_sigma = 0;

  /// Wheel odometry increment since the previous keyframe, in the vehicle frame
  /// of the previous keyframe (planar: x forward, y left, yaw).
  std::optional<mrpt::math::TPose2D> wheel_odom_from_prev;
};

/** A loop closure: pose of keyframe `to` as seen from keyframe `from`. */
struct GlobalGraphLoopEdge
{
  std::size_t from = 0;
  std::size_t to = 0;
  mrpt::poses::CPose3DPDFGaussian relative_pose;
};

struct GlobalGraphInput
{
  std::vector<GlobalGraphKeyframe> keyframes;
  std::vector<GlobalGraphLoopEdge> loops;

  /// ENU origin. If empty, the first GNSS reading is used (and returned).
  std::optional<mrpt::topography::TGeodeticCoords> geodetic_datum;

  /// Initial guess of T_enu_to_map (e.g. the previous solution).
  std::optional<mrpt::poses::CPose3D> T_enu_to_map_guess;
};

struct GlobalGraphResult
{
  /// Optimized vehicle pose in the map frame, one per input keyframe.
  std::vector<mrpt::poses::CPose3D> poses;

  /// Set once observable: enough GNSS factors and a T_enu_to_map azimuth sigma
  /// within the GlobalGraphParams limit.
  std::optional<mola::Georeferencing> georeferencing;

  /// The ENU origin used (input or first GNSS reading).
  std::optional<mrpt::topography::TGeodeticCoords> geodetic_datum;

  std::size_t num_odometry_factors = 0;
  std::size_t num_gnss_factors = 0;
  std::size_t num_gravity_factors = 0;
  std::size_t num_gyro_factors = 0;
  std::size_t num_loop_factors = 0;
  std::size_t num_wheel_odom_factors = 0;
  /// Wheel odometry increments not used: inconsistent with the gyroscope or the
  /// front end (see GlobalGraphParams).
  std::size_t num_wheel_odom_inconsistent = 0;

  /// Azimuth sigma of T_enu_to_map [rad], compared against the observability
  /// limit (if computed).
  std::optional<double> georef_sigma_yaw;

  /// The GNSS readings were not used: their spread is too small for their sigmas.
  bool gnss_degenerate = false;

  double initial_error = 0;
  double final_error = 0;
};

/** Builds the pose graph of all the keyframes (odometry chain, loop closures, GNSS,
 *  gravity, gyroscope and wheel odometry factors, with the GNSS and IMU factor builders of
 *  mola_georeferencing) and optimizes it with Levenberg-Marquardt.
 *
 *  Variables: P(i), the vehicle pose of keyframe i in the map frame, and T(0),
 *  T_enu_to_map. The first keyframe is the gauge (a tight prior at its initial
 *  guess), so the map frame stays the one of the initial guesses.
 */
[[nodiscard]] GlobalGraphResult optimize_global_graph(
  const GlobalGraphInput & in, const GlobalGraphParams & params,
  mrpt::system::COutputLogger * logger = nullptr);

}  // namespace mola::mapper
