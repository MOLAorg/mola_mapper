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
 * @file   Parameters.h
 * @brief  Parameters of mola::mapper::Mapper, loaded from YAML.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>

#include <cstdint>
#include <string>

namespace mola::mapper
{
/** Parameters of the global pose graph (see GlobalGraph.h). */
struct GlobalGraphParams
{
  /** @name Odometry chain (consecutive keyframes of the same source)
     * @{ */
  /// The relative pose sigmas, from the source covariances, are multiplied by this...
  double odometry_uncertainty_multiplier = 1.0;
  /// ... and then these floors are added [m] and [deg]:
  double odometry_additional_noise_xyz = 0.01;
  double odometry_additional_noise_ang_deg = 0.1;
  /// Sigmas growing with the keyframe step (random walk), per sqrt(m): what lets
  /// GNSS and loop closures correct the odometry drift.
  double odometry_noise_xyz_per_sqrt_m = 0.02;
  double odometry_noise_ang_deg_per_sqrt_m = 0.2;
  /** @} */

  /** @name GNSS (CObservationGPS in the keyframes)
     * @{ */
  bool use_gnss = true;
  double gnss_minimum_uncertainty = 0.5;  ///< [m] per-axis floor of the GNSS sigmas
  double gnss_uncertainty_multiplier = 1.0;
  double gnss_max_uncertainty_horiz = 20.0;  ///< [m] readings above this are not used
  double gnss_robust_huber_k = 1.5;
  /** @} */

  /** @name Gravity (averaged accelerometer, CObservationIMU in the keyframes)
     * @{ */
  bool use_imu_gravity = true;
  double imu_gravity_sigma_deg = 3.0;
  /** @} */

  /// Huber parameter of the loop closure edges.
  double loop_closure_robust_huber_k = 1.5;

  /// Gauge: sigma of the prior on the first keyframe [m] and [rad].
  double first_pose_prior_sigma = 1e-4;

  int max_iterations = 100;

  void loadFrom(const mrpt::containers::yaml & cfg);
};

/** Parameters of mola::mapper::Mapper. */
struct Parameters
{
  /// Reference frame of the global map, the same as the state estimator's.
  std::string reference_frame_name = "map";

  /** @name Sensors attached to the keyframes
     * @{ */
  /// Regex of the sensor labels of the GNSS readings to use (empty: none).
  std::string gnss_sensor_labels_re = ".*";
  /// A GNSS reading is attached to the keyframe nearest in time, if within this [s].
  double gnss_max_time_offset = 0.5;

  /// Regex of the sensor labels of the IMU used for gravity (empty: none).
  std::string imu_sensor_labels_re;
  /// Accelerometer readings averaged before each keyframe [s].
  double imu_gravity_window = 1.0;
  /// Minimum readings in that window to attach a gravity observation.
  uint32_t imu_gravity_min_samples = 10;
  /** @} */

  /** @name Global graph optimization
     * @{ */
  GlobalGraphParams graph;
  /// Run the optimizer in a background thread (false: only on optimize_now()).
  bool enable_optimizer_thread = true;
  /// Minimum period between solves [s].
  double optimizer_min_period = 1.0;
  /** @} */

  /** @name Feedback to the state estimator (mola::NavStateFilter)
     * @{ */
  /** If true, after each solve the optimized pose of the newest keyframe is
     *  fused into the state estimator as an absolute pose in reference_frame_name,
     *  so loop closures and GNSS reach the live estimate. */
  bool feedback_to_state_estimator = false;
  /// Floors of the sigmas of that pose [m] and [deg].
  double feedback_min_sigma_xyz = 0.10;
  double feedback_min_sigma_ang_deg = 0.5;
  /** @} */

  /** @name Loop closure (mola_sm_loop_closure engine, detector only)
     * @{ */
  bool loop_closure_enabled = false;
  /// Scans driven by a background thread (false: by the caller).
  bool enable_loop_closure_thread = true;
  std::string loop_closure_pipeline_file;
  double loop_closure_check_period_sec = 5.0;  ///< [s]
  uint32_t loop_closure_min_new_keyframes = 10;
  bool loop_closure_incremental = true;
  uint32_t loop_closure_full_scan_every_n = 10;
  /// End-of-run passes (full scan + re-optimization) until no new loop.
  uint32_t loop_closure_finalize_rounds = 4;
  /** @} */

  /** @name Outputs, written at the end (empty: not written)
     * @{ */
  /// Keyframe poses in reference_frame_name, TUM format.
  std::string output_tum_trajectory;
  /// Simplemap with the optimized keyframes and all their observations.
  std::string output_simplemap;
  /// Geo-reference (YAML: datum, T_enu_to_map and its covariance).
  std::string output_georeference;
  /// Loop closure edges, one per line: "kf_from kf_to x y z yaw pitch roll".
  std::string output_loop_closures;
  /** The trajectory of the state estimator (its estimated_trajectory(), e.g.
   *  the smoother with keep_finalized_trajectory) and its geo-reference, for
   *  comparisons. */
  std::string output_state_estimator_tum;
  std::string output_state_estimator_georeference;
  /** Prefix of the files with the global graph input (see GlobalGraphIO.h), to
   *  re-optimize it offline with other parameters (mola-mapper-reoptimize). */
  std::string output_graph_input;
  /** @} */

  void loadFrom(const mrpt::containers::yaml & cfg);
};

}  // namespace mola::mapper
