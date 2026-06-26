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
 * @brief  Parameters for the Mapper3D central 3D SLAM module.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>
#include <optional>
#include <string>

namespace mola::mapper_3d
{
/** Kinematic motion model used to build the inter-keyframe factors. */
enum class KinematicModel : uint8_t
{
  ConstantVelocity,  //!< Hand-held / flying: full SE(3) constant velocity.
  Tricycle,          //!< Ground vehicle: Ackermann / tricycle constraints.
};

/** All configuration parameters of Mapper3D, loaded from a YAML map node.
 *
 * The navigation-state fusion group mirrors
 * `mola::state_estimation_smoother::Parameters` so existing tuning carries over.
 *
 * \ingroup mola_mapper_3d_grp
 */
class Parameters
{
public:
  Parameters() = default;

  /// Loads all parameters from a YAML map node (the `params:` subtree).
  void loadFrom(const mrpt::containers::yaml & cfg);

  /** @name Reference frame IDs
   * @{ */

  /// Vehicle/robot base frame, used when publishing timely pose updates.
  std::string vehicle_frame_name = "base_link";

  /// Reference frame for pose publication. Typically 'map' or 'enu'.
  std::string reference_frame_name = "map";

  /// The ENU geo-reference frame.
  std::string enu_frame_name = "enu";

  /** @} */

  /** @name Kinematic factors and keyframe creation (motion model)
   * @{ */

  KinematicModel kinematic_model = KinematicModel::ConstantVelocity;

  /// Valid estimations are extrapolated only up to this time since the last
  /// incorporated observation; farther queries return std::nullopt.
  double max_time_to_use_velocity_model = 2.0;  // [s]

  /// Minimum time difference between two keyframes to create a new one.
  double min_time_difference_to_create_new_frame = 0.01;  // [s]

  /// Warn (but keep going) if the gap between two keyframes exceeds this.
  double time_between_frames_to_warning = 3.0;  // [s]

  /// Permissive temporal tolerance to attach GNSS factors to a nearby keyframe.
  double gnss_nearby_keyframe_stamp_tolerance = 1.0;  // [s]

  /// Permissive temporal tolerance to attach IMU attitude/gravity factors.
  double imu_nearby_keyframe_stamp_tolerance = 0.10;  // [s]

  /// Maximum rate [Hz] of SUMMARIZED IMU observations actually inserted into the
  /// graph (0 = disabled = insert every reading). When > 0, the high-rate IMU
  /// stream is buffered and at most this many summarized observations per second
  /// are fused: each carries the AVERAGED accelerometer (a less-noisy
  /// gravity/leveling estimate), the averaged angular velocity, and the latest
  /// absolute orientation, instead of just dropping the in-between samples. This
  /// bounds BOTH the inserted-factor rate AND the IMU-driven keyframe-creation
  /// rate (the keyframe-reuse window becomes 1/rate), keeping the central graph
  /// tractable for real ~100-400 Hz IMUs without a fixed decimation ratio.
  double imu_max_insert_rate_hz = 5.0;  // [Hz]

  /// Maximum rate [Hz] of wheel-odometry increments inserted into the graph
  /// (0 = disabled = insert every reading). Readings arriving sooner than 1/rate
  /// after the last kept one are dropped WITHOUT advancing the integration anchor
  /// (so the next kept reading fuses the full accumulated motion + covariance);
  /// no information is thrown away, only the factor/keyframe cadence is bounded.
  double odometry_max_insert_rate_hz = 5.0;  // [Hz]

  /// When true, high-rate IMU and wheel odometry do NOT each spawn their own
  /// keyframe. They share keyframes created at a bounded cadence
  /// (sensor_keyframe_min_period), and wheel odometry is aggregated into a
  /// single relative-pose edge between consecutive keyframes (the "consecutive
  /// frame edge" model) instead of one absolute factor per sample. This bounds
  /// central-graph growth from direct high-rate sensors. Keyframes requested by
  /// LIO/VIO via SharedKeyframeMap are unaffected (still inserted as requested);
  /// dense per-scan LIO/VIO fuse_pose() is a separate concern (predictor
  /// separation, plan 6.2b).
  bool aggregate_high_rate_into_edges = false;

  /// Keyframe "close enough" window for direct high-rate sensors when
  /// aggregate_high_rate_into_edges is on: a sample within this period of an
  /// existing keyframe reuses it (a common coarse keyframe clock shared by IMU
  /// and wheels), so direct-sensor keyframes are created at most ~1/this rate.
  double sensor_keyframe_min_period = 0.5;  // [s]

  double sigma_random_walk_acceleration_linear = 1.0;   // [m/s^2]
  double sigma_random_walk_acceleration_angular = 1.0;  // [rad/s^2]
  double sigma_integrator_position = 0.10;              // [m]
  double sigma_integrator_orientation = 0.10;           // [rad]

  double sigma_twist_from_consecutive_poses_linear = 1.0;   // [m/s]
  double sigma_twist_from_consecutive_poses_angular = 1.0;  // [rad/s]

  mrpt::math::TTwist3D initial_twist;
  double initial_twist_sigma_lin = 0.1;  // [m/s]
  double initial_twist_sigma_ang = 0.1;  // [rad/s]

  bool enforce_planar_motion = false;

  /// If set, the first frame gets an SE(3) prior toward the reference origin
  /// with this sigma. Use a small number (e.g. 1e-6) for pure-odometry runs or
  /// when estimating geo-referencing. Leave empty for pre-built geo-ref maps.
  std::optional<double> link_first_pose_to_reference_origin_sigma;

  /** @} */

  /** @name Keyframe ingestion (SharedKeyframeMap sink)
   * @{ */

  /// Noise of the tight, consecutive-keyframe relative-pose factor chaining
  /// requestInsertKeyframe() calls from the same source (2.8 drift-fix design:
  /// local LIO/VIO accuracy, NOT the absolute drift of the source over time).
  /// Divided by the request's `quality` field (clamped away from zero).
  double keyframe_ingestion_sigma_lin = 0.02;     // [m]
  double keyframe_ingestion_sigma_ang_deg = 0.5;  // [deg]

  /** @} */

  /** @name IMU related
   * @{ */

  double imu_attitude_sigma_deg = 2.0;
  double imu_attitude_azimuth_offset_deg = 0.0;
  /// Sigma to estimate the up-vector from accelerometer (gravity alignment).
  /// Set to 0 to disable.
  double imu_normalized_gravity_alignment_sigma = 0.4;
  /// If > 0 and the IMU provides angular velocity, add a body-frame gyro prior
  /// on the keyframe's W variable with this sigma [deg/s]. 0 disables it (the
  /// inter-keyframe angular-velocity integration factor still runs).
  double imu_angular_velocity_sigma_deg = 0.0;

  /** @} */

  /** @name Geo-referencing
   * @{ */

  bool estimate_geo_reference = false;
  std::optional<mola::Georeferencing> fixed_geo_reference;
  double convergence_max_position_sigma = 1.0;         // [m]
  double convergence_max_orientation_sigma_deg = 5.0;  // [deg]
  bool publish_estimated_georef_on_convergence = true;
  double gnss_huber_threshold = 1.5;  // [sigmas]

  /** @} */

  /** @name Nonlinear optimization
   * @{ */

  /// Each new keyframe triggers an iSAM2 update plus this many refining steps.
  uint32_t additional_isam2_update_steps = 3;

  /// If true, the (growing) iSAM2 solve runs on a dedicated background thread,
  /// so estimated_navstate() and the high-rate publisher return immediately
  /// from cached estimates instead of paying the solve cost on the calling
  /// thread. Recommended for any real online run. Keep false for deterministic
  /// single-threaded use (e.g. unit tests), where every query first flushes and
  /// solves synchronously so it reflects all data fused so far.
  bool enable_optimizer_thread = false;

  /** @} */

  /** @name High-rate pose publisher
   * @{ */

  /// Rate at which to publish the extrapolated reference-frame pose. 0=disabled.
  double high_rate_pose_publish_rate_hz = 0.0;  // [Hz]

  /// If true, fold the latest IMU/wheels readings (since the anchor) into the
  /// extrapolation; otherwise extrapolate the anchor with the kinematic model.
  bool high_rate_use_latest_sensors = true;

  /** @} */

  /** @name Map save/load (single externalized .simplemap)
   * @{ */

  /// If non-empty, load this .simplemap at startup (localization / continue).
  std::string load_simplemap_file;

  /// If non-empty, save the map here on shutdown.
  std::string save_simplemap_file;

  /// Seconds a keyframe's raw observations stay in RAM before being
  /// externalized (flushed to the sidecar directory). 0 = never externalize.
  double externalize_after_seconds = 30.0;  // [s]

  /** @} */

  /** @name Sensor input names (regex)
   * @{ */

  std::string do_process_imu_labels_re = ".*";
  std::string do_process_odometry_labels_re = ".*";
  std::string do_process_gnss_labels_re = ".*";

  /** @} */
};

}  // namespace mola::mapper_3d

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola::mapper_3d, mola::mapper_3d::KinematicModel)
MRPT_FILL_ENUM(KinematicModel::ConstantVelocity);
MRPT_FILL_ENUM(KinematicModel::Tricycle);
MRPT_ENUM_TYPE_END()
