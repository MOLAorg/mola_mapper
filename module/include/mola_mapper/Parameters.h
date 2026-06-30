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
 * @brief  Parameters for the Mapper central 3D SLAM module.
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

namespace mola::mapper
{
/** Kinematic motion model used to build the inter-keyframe factors. */
enum class KinematicModel : uint8_t
{
  ConstantVelocity,  //!< Hand-held / flying: full SE(3) constant velocity.
  Tricycle,          //!< Ground vehicle: Ackermann / tricycle constraints.
};

/** Controls which code paths are allowed to create new keyframe variables in
 *  the GTSAM graph (plan 4.13 Phase A).
 *
 *  - Auto: acts like SharedMapOnly once any requestInsertKeyframe() call is
 *    received; before that, falls back to legacy creation (all paths create KFs
 *    at min_time_difference_to_create_new_frame). Suitable for typical
 *    LIO+mapper3d deployments: the first sparse KF from LIO flips the mode.
 *  - SharedMapOnly: ONLY requestInsertKeyframe() creates keyframe GTSAM
 *    variables. Dense fuse_pose(), fuse_imu(), fuse_odometry(), fuse_gnss()
 *    only record the predictor anchor and snap to the nearest EXISTING keyframe
 *    for sensor factors. Requires a SharedKeyframeMap producer (LIO/VIO).
 *  - SensorClock: sensor paths create KFs at a bounded rate governed by
 *    sensor_clock_min_period_s. For sensor-only runs (wheels + IMU + GNSS
 *    with NO LIO/VIO front end).
 */
enum class KeyframeCreationSource : uint8_t
{
  Auto,           //!< Legacy until first requestInsertKeyframe(); then SharedMapOnly.
  SharedMapOnly,  //!< Only requestInsertKeyframe() creates KF graph variables.
  SensorClock,    //!< Sensor paths create KFs at sensor_clock_min_period_s rate.
};

/** All configuration parameters of Mapper, loaded from a YAML map node.
 *
 * The navigation-state fusion group mirrors
 * `mola::state_estimation_smoother::Parameters` so existing tuning carries over.
 *
 * \ingroup mola_mapper_grp
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

  /// How keyframe-variable creation is gated (plan 4.13 Phase A). Default
  /// 'auto' keeps existing tests passing while automatically switching to
  /// SharedMapOnly behavior once an LIO/VIO SharedKeyframeMap producer is
  /// detected (i.e. after the first requestInsertKeyframe() call).
  KeyframeCreationSource keyframe_creation_source = KeyframeCreationSource::Auto;

  /// Minimum time [s] between keyframes created by the SENSOR CLOCK path
  /// (SensorClock mode). Governs the coarse KF clock for sensor-only runs
  /// (wheels + IMU + GNSS without LIO/VIO).
  double sensor_clock_min_period_s = 0.5;  // [s]

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

  /// Consecutive-keyframe odometry-edge noise is derived from the PROPAGATED
  /// relative-pose covariance of the two source pose PDFs (cov_to (-) cov_from),
  /// mirroring mola_sm_loop_closure::add_odometry_edges. The per-DOF sigma is
  /// sqrt(diag) * odometry_edge_uncertainty_multiplier, with a small additive
  /// floor below. This makes the edge anisotropic: tight in the well-observed
  /// DOFs (x/y/yaw) and soft in the drift-prone ones (z/roll/pitch) so IMU
  /// gravity / GNSS can level the map. Falls back to the floor alone when the
  /// source provides no (or zero) covariance. Defaults match
  /// mola_sm_loop_closure::OdometryEdgeParams (multiplier 1.0, additive floor
  /// 1e-3 m / 1e-3 deg).
  double odometry_edge_uncertainty_multiplier = 1.0;
  double odometry_edge_min_sigma_xyz = 1e-3;      // [m]   additive floor
  double odometry_edge_min_sigma_ang_deg = 1e-3;  // [deg] additive floor
  /// If > 0, RAISES the odometry-edge roll/pitch floor to this many degrees
  /// (yaw stays at odometry_edge_min_sigma_ang_deg), making the orientation
  /// chain compliant in roll/pitch so an IMU gravity factor can level the map.
  /// 0 disables it. ONLY enable together with the IMU gravity-leveling factor;
  /// otherwise roll/pitch would drift unconstrained.
  double odometry_edge_min_sigma_rollpitch_deg = 0.0;

  /** @} */

  /** @name IMU related
   * @{ */

  /// Absolute-attitude (Pose3RotationFactor) sigma [deg]. Set to 0 to disable
  /// the attitude factor. This factor is added at every keyframe whenever the
  /// IMU provides an orientation quaternion, REGARDLESS of geo-referencing: the
  /// absolute attitude is itself an azimuth reference, so it lets iSAM2 estimate
  /// the geo-reference (T_enu_to_map yaw) automatically even with GNSS off (the
  /// no-geo-ref F0 prior is yaw-free; see Mapper::reinitialize_gtsam_locked).
  double imu_attitude_sigma_deg = 2.0;
  double imu_attitude_azimuth_offset_deg = 0.0;
  /// If > 0 and the IMU provides angular velocity, add a body-frame gyro prior
  /// on the keyframe's W variable with this sigma [deg/s] (built from the
  /// interval-averaged angular velocity). 0 disables it.
  double imu_angular_velocity_sigma_deg = 0.0;

  /// IMU gravity-leveling reducer (see agents.md "IMU gravity leveling"). The
  /// proper-acceleration samples accumulated between two keyframes are pooled,
  /// motion-contaminated ones (|a| far from g, high |w|) rejected, and ONE
  /// MeasuredGravityFactor with a DATA-EARNED sigma is emitted on the keyframe.
  /// Length of the gravity-pooling window [s]. ~1 s @ 100 Hz IMU -> ~100 samples.
  double imu_gravity_window_sec = 1.0;
  /// Accept a sample only if | ‖a‖ - g | <= this fraction of g.
  double imu_gravity_accel_tol_frac = 0.05;
  /// Accept a sample only if the angular rate magnitude is below this [deg/s].
  double imu_gravity_gyro_tol_deg = 3.0;
  /// Minimum accepted samples in a window to emit a gravity factor.
  std::size_t imu_gravity_min_samples = 10;
  /// Reject a window whose accepted samples disagree by more than this RMS
  /// angular spread [deg] (motion-contaminated; would tilt the map). <=0 off.
  double imu_gravity_max_spread_deg = 3.0;
  /// Reject a window unless at least this fraction of raw samples passed the
  /// low-dynamics gates (a hard-accel/turning epoch accepts very few). <=0 off.
  /// Off by default: a high-vibration IMU accepts only a few percent of samples
  /// even when stationary, so this would block the clean stops too; the
  /// spread gate is the robust discriminator. Useful for low-noise IMUs.
  double imu_gravity_min_accept_fraction = 0.0;
  /// Lower / upper clamp on the earned gravity-factor sigma [deg].
  double imu_gravity_sigma_floor_deg = 0.5;
  double imu_gravity_sigma_ceil_deg = 5.0;

  /** @} */

  /** @name Geo-referencing
   * @{ */

  bool estimate_geo_reference = false;
  std::optional<mola::Georeferencing> fixed_geo_reference;
  /// Isotropic prior sigma used to PIN T_enu_to_map to identity when neither
  /// estimate_geo_reference nor fixed_geo_reference is set (pure IMU/odometry).
  /// Tight so {map}=={enu} and IMU gravity levels the {map} keyframes directly
  /// (units: rad for rotation DOFs / m for translation; see agents.md).
  double enu_to_map_prior_sigma_no_georef = 1e-3;
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

  /** @name Output trajectory
   * @{ */

  /// If non-empty, save the estimated robot trajectory (in {reference_frame})
  /// as a TUM-format file at shutdown, and on demand via the GUI.
  /// Set via env var MOLA_MAPPER3D_TUM_TRAJECTORY_OUTPUT (empty = don't save).
  std::string save_trajectory_to_file;

  /** @} */
};

}  // namespace mola::mapper

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola::mapper, mola::mapper::KinematicModel)
MRPT_FILL_ENUM(KinematicModel::ConstantVelocity);
MRPT_FILL_ENUM(KinematicModel::Tricycle);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola::mapper, mola::mapper::KeyframeCreationSource)
// Short lower-case aliases accepted in YAML; canonical C++ name registered last
// so enum->string (logs/diagnostics) returns the full scoped name.
MRPT_FILL_ENUM_CUSTOM_NAME(KeyframeCreationSource::Auto, "auto");
MRPT_FILL_ENUM_CUSTOM_NAME(KeyframeCreationSource::SharedMapOnly, "shared_map_only");
MRPT_FILL_ENUM_CUSTOM_NAME(KeyframeCreationSource::SensorClock, "sensor_clock");
MRPT_FILL_ENUM(KeyframeCreationSource::Auto);
MRPT_FILL_ENUM(KeyframeCreationSource::SharedMapOnly);
MRPT_FILL_ENUM(KeyframeCreationSource::SensorClock);
MRPT_ENUM_TYPE_END()
