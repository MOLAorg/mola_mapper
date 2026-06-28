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
 * @file   Mapper3D.h
 * @brief  Central 3D SLAM map: online multi-sensor fusion, loop closure and
 *         geo-referencing into one optimized world model.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_mapper_3d/ImuGravityFilter.h>
#include <mola_mapper_3d/Parameters.h>
#include <mola_mapper_3d/WorldModelState.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gtsam
{
class Values;
}  // namespace gtsam

namespace mola::mapper_3d
{
/** Central 3D SLAM map and multi-sensor fusion backend.
 *
 * Mapper3D holds one global, optimized representation of the world, based on
 * keyframes (a CSimpleMap with raw observations and annotations) and a GTSAM
 * factor graph. It fuses multiple odometry sources (wheels, LiDAR-odometry,
 * visual-odometry), IMU, and GNSS, and provides:
 *  - `mola::NavStateFilter`: the single source of truth for short-term pose
 *    prediction queried by LIO/VIO front ends.
 *  - `mola::LocalizationSourceBase`: publishes the fused vehicle localization.
 *  - `mola::MapSourceBase`: publishes the map / geo-referencing.
 *  - `mola::DiagnosticsProvider`: structured diagnostics.
 *
 * Frame conventions mirror `mola_state_estimation_smoother`: one frame per
 * odometry source ({odom_i}), plus the internal {map} and {enu} frames; the
 * graph estimates `T_map_to_odom_i` per source and `T_enu_to_map` for
 * geo-referencing.
 *
 * \ingroup mola_mapper_3d_grp
 */
class Mapper3D : public mola::NavStateFilter,
                 public mola::LocalizationSourceBase,
                 public mola::MapSourceBase,
                 public mola::DiagnosticsProvider,
                 public mola::SharedKeyframeMap
{
  DEFINE_MRPT_OBJECT(Mapper3D, mola::mapper_3d)

public:
  Mapper3D();
  ~Mapper3D() override;

  // Non-copyable / non-movable (owns GTSAM state).
  Mapper3D(const Mapper3D &) = delete;
  Mapper3D & operator=(const Mapper3D &) = delete;
  Mapper3D(Mapper3D &&) = delete;
  Mapper3D & operator=(Mapper3D &&) = delete;

  /** @name Main API
   * @{ */

  /// Read-only access to parameters (set once via initialize()).
  [[nodiscard]] const Parameters & parameters() const { return params_; }

  void initialize(const mrpt::containers::yaml & cfg) override;
  void spinOnce() override;
  void reset() override;

  // --- NavStateFilter sensor inputs ---
  void fuse_pose(
    const mrpt::Clock::time_point & timestamp, const mrpt::poses::CPose3DPDFGaussian & pose,
    const std::string & frame_id) override;

  void fuse_odometry(
    const mrpt::obs::CObservationOdometry & odom,
    const std::string & odomName = "odom_wheels") override;

  void fuse_imu(const mrpt::obs::CObservationIMU & imu) override;

  void fuse_gnss(const mrpt::obs::CObservationGPS & gps) override;

  void fuse_twist(
    const mrpt::Clock::time_point & timestamp, const mrpt::math::TTwist3D & twist,
    const mrpt::math::CMatrixDouble66 & twistCov) override;

  [[nodiscard]] std::optional<NavState> estimated_navstate(
    const mrpt::Clock::time_point & timestamp, const std::string & frame_id) override;

  // --- Diagnostics / convergence ---
  [[nodiscard]] bool has_converged_localization(
    mrpt::poses::CPose3DPDFGaussian & pose_in_map) const override;

  // --- RawDataConsumer ---
  void onNewObservation(const CObservation::ConstPtr & o) override;

  // --- DiagnosticsProvider ---
  void getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status) override;

  // --- SharedKeyframeMap ---
  std::optional<SharedKeyframeMap::KeyFrameID> requestInsertKeyframe(
    const SharedKeyframeMap::KeyframeInsertRequest & req) override;

  /** @} */

  /** @name Introspection (diagnostics / tests)
   * @{ */

  /// Known odometry frame_ids registered so far.
  [[nodiscard]] std::set<std::string> known_odometry_frame_ids() const;

  /// Number of keyframes currently in the central map.
  [[nodiscard]] std::size_t keyframe_count() const;

  /// Latest estimated T_map_to_odometry_frame for a given frame_id, if known.
  [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_map_to_odometry_frame(
    const std::string & frame_id) const;

  /// Latest estimated T_enu_to_map (geo-referencing), if available.
  [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_enu_to_map() const;

  /// Current geo-referencing (fixed or estimated), if available.
  [[nodiscard]] std::optional<mola::Georeferencing> current_georeferencing() const;

  /** @} */

private:
  Parameters params_;
  bool params_loaded_ = false;

  WorldModelState state_;
  mutable std::recursive_mutex stateMutex_;

  // --- Background optimizer thread (so queries return from cached estimates
  //     instead of paying the growing iSAM2 solve cost; see optimize_and_refresh
  //     and the plan's "quickly returning queries / high-rate publisher" goal).
  std::thread optimizer_thread_;
  std::mutex optimizer_wakeup_mutex_;
  std::condition_variable optimizer_wakeup_cv_;
  bool optimizer_pending_ = false;  // guarded by optimizer_wakeup_mutex_
  std::atomic_bool optimizer_should_exit_{false};
  // Serializes the actual iSAM2 solve (background thread vs. a synchronous
  // flush), so only one thread ever touches the iSAM2/estimate objects.
  std::mutex solve_mutex_;
  // Throttle anchor for the high-rate publisher (wall-clock).
  std::optional<mrpt::Clock::time_point> last_publish_wallclock_;

  // --- Keyframe-creation gating (plan 4.13 Phase A) ---
  // Set to true the first time requestInsertKeyframe() is called. In Auto mode
  // this flips the behavior from legacy (create from all sensor paths) to
  // SharedMapOnly (only requestInsertKeyframe() creates KF variables).
  bool shared_kf_producer_active_ = false;

  // Wheel-odometry integration anchor (last absolute reading per single source).
  std::optional<mrpt::poses::CPose2D> last_wheels_odometry_;
  std::optional<std::string> last_wheels_odometry_name_;
  // Stamp of the last *kept* (not rate-capped) wheel-odometry reading.
  std::optional<mrpt::Clock::time_point> last_wheels_odometry_stamp_;

  // --- High-rate IMU max-rate summarization (imu_max_insert_rate_hz) ---
  // Buffers incoming IMU samples and inserts at most imu_max_insert_rate_hz
  // SUMMARIZED observations/second (averaged accel + gyro, latest orientation),
  // bounding both the factor and the IMU-keyframe creation rate. See fuse_imu().
  struct ImuAccumulator
  {
    std::size_t n_acc = 0;
    std::size_t n_gyro = 0;
    std::array<double, 3> acc_sum = {0, 0, 0};
    std::array<double, 3> gyro_sum = {0, 0, 0};
    bool has_quat = false;
    std::array<double, 4> quat_wxyz = {1, 0, 0, 0};  //!< latest absolute orientation
    mrpt::poses::CPose3D sensor_pose;
    mrpt::Clock::time_point last_stamp;
    [[nodiscard]] bool empty() const { return n_acc == 0 && n_gyro == 0 && !has_quat; }
    void clear() { *this = ImuAccumulator{}; }
  };
  ImuAccumulator imu_accum_;
  std::optional<mrpt::Clock::time_point> last_imu_summary_stamp_;

  // --- Filtered low-dynamics gravity leveling (imu_use_filtered_gravity) ---
  // Accumulates the RAW high-rate accelerometer/gyro stream, rejects samples
  // contaminated by vehicle acceleration/rotation, and emits ONE strong
  // MeasuredGravityFactor per window with a data-earned sigma. This replaces the
  // per-sample gravity factor, whose ~2 deg random motion noise could never
  // level the map (see agents.md "IMU gravity leveling"). Guarded by stateMutex_.
  ImuGravityFilter imu_gravity_filter_;
  mrpt::poses::CPose3D last_imu_sensor_pose_;

  // --- Geo-referencing diagnostics counters (guarded by stateMutex_) ---
  std::size_t gnss_factors_inserted_ = 0;
  std::size_t gnss_readings_seen_ = 0;
  std::size_t imu_factors_inserted_ = 0;
  bool georef_converged_announced_ = false;

  // Wheel-odometry relative-chaining aggregation (aggregate_high_rate_into_edges):
  // the keyframe the wheel chain last attached to, and the absolute wheel
  // odometry reading at that keyframe. On a keyframe transition, ONE relative
  // edge Between(T(last_kf), T(new_kf)) is emitted using the net wheel motion
  // since the anchor (no per-sample keyframe / absolute factor).
  std::optional<KeyFrameID> wheel_chain_last_kf_;
  std::optional<mrpt::poses::CPose2D> wheel_chain_anchor_odom_;

  // Phase B.1: wheel relative-pose edges between consecutive SPARSE keyframes
  // (created via requestInsertKeyframe()). When a new sparse KF is closed, the
  // net wheel motion since the previous sparse KF is emitted as a
  // BetweenFactor(T(prev_shared_kf), T(new_kf)) using the motion-model
  // covariance. Fires in any keyframe_creation_source mode whenever wheel data
  // has been accumulated between consecutive requestInsertKeyframe() calls.
  // Cleared in reset_sensor_anchors_locked().
  std::optional<KeyFrameID> prev_shared_kf_id_;
  std::optional<mrpt::poses::CPose2D> wheel_odom_at_prev_shared_kf_;

  // Per-source bookkeeping for keyframe-insertion requests (SharedKeyframeMap
  // sink): the last keyframe id and the front end's own pose_in_source mean it
  // was inserted with, used to chain consecutive requests via their
  // *relative* motion (see 2.8 drift-fix design /
  // request_insert_keyframe_locked()). The request's covariance is NOT used:
  // the chain/anchor factors always use our own configured
  // keyframe_ingestion_sigma_* noise, not the front end's self-reported
  // uncertainty (which can be pathologically tiny, e.g. a relocalization seed).
  // --- Odometry backbone: a single CONSECUTIVE relative-pose-edge chain ---
  // Both the dense fuse_pose() path and the sparse SharedKeyframeMap sink feed
  // odometry into ONE shared chain built exactly like
  // mola_sm_loop_closure::add_odometry_edges: each keyframe stores the absolute
  // odometry pose that defined it, and every pair of TIME-ADJACENT keyframes
  // (regardless of which source created them) is linked by exactly ONE relative
  // BetweenFactor(T(prev), T(next)) = pose_next (-) pose_prev. This replaces the
  // previous design (absolute Between(F(odom_i), T(kf)) ties AND per-source
  // independent chains): a single rigid T_map_to_odom_i cannot fit the whole
  // central map once odometry drifts, and per-source chains skip each other's
  // keyframes, both producing globally inconsistent / conflicting edges that
  // deformed {map} (catastrophic z/tilt). Consecutive-only edges keep the
  // backbone a consistent tree, leaving global z/tilt soft so IMU-gravity /
  // GNSS / loop-closure factors can override it (plan 2.8).
  // Stores the full pose PDF (mean + covariance) so the consecutive-keyframe
  // relative-pose edges can derive their per-DOF, anisotropic noise from the
  // propagated relative covariance (cov_to (-) cov_from), as
  // mola_sm_loop_closure::add_odometry_edges does. This leaves the drift-prone
  // DOFs (z, roll, pitch) appropriately soft so IMU-gravity / GNSS can level
  // the map, instead of pinning every edge to a hardcoded isotropic sigma.
  std::map<KeyFrameID, mrpt::poses::CPose3DPDFGaussian> kf_odom_abs_pose_;
  std::set<std::pair<KeyFrameID, KeyFrameID>> odom_chain_edges_;  // (from<to by time)
  std::set<OdometryFrameID> odom_frame_anchored_;

  // Latest keyframe each odometry source contributed an absolute pose to. Used
  // to report T_map_to_odom_i as the INSTANTANEOUS transform aligning the
  // source's latest odom pose with its keyframe's optimized {map} pose
  // (map_pose(kf) (+) inverse(odom_abs(kf))), rather than the persistent
  // F(odom_i) graph variable. F(odom_i) is now only a one-time gauge anchor (the
  // odometry backbone is a relative-pose chain, not per-keyframe absolute ties),
  // so it no longer tracks the live map<-odom drift; this does. Consumed by the
  // GUI drift readout and the per-source movable viz frame placement.
  std::map<OdometryFrameID, KeyFrameID> latest_kf_by_odom_frame_;

  // Compiled sensor-label filters (built in initialize() from the params regex).
  std::regex imu_labels_re_;
  std::regex odometry_labels_re_;
  std::regex gnss_labels_re_;

  // --- Visualization (optional MolaViz / MolaVizImGui via VizInterface) ---
  // Discovered in initialize() via findService<VizInterface>(); null if no
  // visualizer module is loaded (e.g. headless runs / unit tests).
  mola::VizInterface::Ptr visualizer_;
  // Raw "visualization" YAML sub-section (sibling of "params"); keys read
  // on demand, mirroring mola_mapper_2d / mola_lidar_odometry.
  mrpt::containers::yaml viz_params_ = mrpt::containers::yaml::Map();
  std::mutex state_gui_mtx_;
  bool gui_created_ = false;
  std::optional<mrpt::Clock::time_point> last_viz_update_wallclock_;

  // Runtime view toggles, driven from the GUI "Control" tab. Written from the
  // GUI thread, read from the spinOnce/viz thread (atomic to avoid a data race;
  // a stale-by-one-frame read only delays a checkbox by one update).
  std::atomic_bool viz_show_keyframes_{true};
  std::atomic_bool viz_show_edges_{true};
  std::atomic_bool viz_show_odom_frames_{true};
  std::atomic_bool viz_camera_follows_vehicle_{false};

  /// Which frame is the origin (0,0,0) of the 3D viz scene: 0 = {map},
  /// 1 = {enu}. Default {enu} so the map is rendered North-oriented via the
  /// (estimated or identity) T_enu_to_map. Selected via the GUI "View" tab.
  std::atomic_int viz_reference_frame_{1};

  // LiveStrings shared between the module (writer) and the GUI (reader).
  struct GuiData
  {
    mola::gui::LiveString::Ptr lbKeyframes;
    mola::gui::LiveString::Ptr lbEdges;
    mola::gui::LiveString::Ptr lbOdomFrames;
    mola::gui::LiveString::Ptr lbGeoref;
    mola::gui::LiveString::Ptr lbConverged;
    mola::gui::LiveString::Ptr lbGnss;
    mola::gui::LiveString::Ptr lbEnu;
    mola::gui::LiveString::Ptr lbDrift;
    mola::gui::LiveString::Ptr lbImu;
  };
  GuiData gui_;

  /// Renders the keyframe tree, graph edges and per-source movable {odom_i}
  /// frames into the visualizer, and creates/updates the GUI sub-window.
  /// Throttled to visualization.update_rate_hz. No-op when no visualizer is
  /// attached. (Mapper3D_GUI.cpp)
  void updateVisualization();

  /// Builds the backend-agnostic GUI sub-window (status + control tabs) once.
  void internalBuildGUI();

  // --- helpers implemented across the Mapper3D_*.cpp translation units ---

  /// Registers (or returns the existing id for) an odometry frame name.
  [[nodiscard]] OdometryFrameID add_or_get_odom_frame_id_locked(const std::string & frame_id_name);

  /// Creates a new keyframe for timestamp `t`, or returns the existing one when
  /// close enough. Carries the out-of-order keyframe guard: a request older than
  /// the newest keyframe snaps to the nearest existing keyframe.
  [[nodiscard]] KeyFrameID create_or_get_keyframe_by_timestamp_locked(
    const mrpt::Clock::time_point & t,
    const std::optional<double> & overrideCloseEnough = std::nullopt);

  using stamp_map_t = std::map<mrpt::Clock::time_point, KeyFrameID>;
  using pair_nearby_frame_iterators_t =
    std::pair<stamp_map_t::const_iterator, stamp_map_t::const_iterator>;

  /// Links a keyframe into the single consecutive odometry-edge chain: stores
  /// its absolute odometry pose (first writer wins), anchors the source frame
  /// F(frameIdx) once, and adds a relative BetweenFactor to each time-adjacent
  /// keyframe that also has a stored odometry pose. See kf_odom_abs_pose_.
  void link_into_odometry_chain_locked(
    KeyFrameID kf, const mrpt::poses::CPose3DPDFGaussian & absOdomPosePdf,
    OdometryFrameID frameIdx);

  /// Adds (once) the consecutive relative-pose edge between two time-adjacent
  /// keyframes from their stored absolute odometry poses.
  void add_odom_chain_edge_locked(KeyFrameID a, KeyFrameID b);

  /// (Re)builds the GTSAM/iSAM2 state and the persistent T_enu_to_map variable.
  void reinitialize_gtsam_locked();

  /// Clears the per-source high-rate integration anchors (wheel odometry,
  /// IMU/publish stamps, keyframe-ingestion chains, wheel aggregation chain).
  void reset_sensor_anchors_locked();

  /// Core pose-fusion entry: adds a prior (reference frame) or a between-factor
  /// (odometry frame) relating F(frame) and T(keyframe).
  void fuse_pose_locked(
    const mrpt::Clock::time_point & timestamp, const mrpt::poses::CPose3DPDFGaussian & pose,
    const std::string & frame_id);

  /// Core keyframe-insertion entry (SharedKeyframeMap::requestInsertKeyframe()):
  /// creates/reuses the keyframe, merges its raw observations, and links it to
  /// the source either with a tight Between(F(i), T(kf)) anchor (first request
  /// of this source) or a tight Between(T(prev_kf), T(kf)) consecutive-frame
  /// relative-pose factor (every later request), per the 2.8 drift-fix design.
  [[nodiscard]] KeyFrameID request_insert_keyframe_locked(
    const SharedKeyframeMap::KeyframeInsertRequest & req);

  /// Creates the T/V/W GTSAM variables for a brand-new keyframe and the priors
  /// (first frame) or seeds initial values from the nearest neighbor.
  void initialize_new_frame(KeyFrameID id, const pair_nearby_frame_iterators_t & closestFrames);

  /// Adds kinematic factors between two time-adjacent keyframes (once).
  void add_kinematic_factor_between(KeyFrameID from, KeyFrameID to);

  /// Adds an IMU observation's attitude / gravity-leveling / gyro factors to a
  /// keyframe selected by the given keyframe-reuse tolerance. Shared by the
  /// per-sample and the summarized (max-rate) IMU paths.
  void apply_imu_observation_locked(
    const mrpt::obs::CObservationIMU & imu, double keyframe_reuse_tolerance);

  /// Emits ONE MeasuredGravityFactor from a filtered low-dynamics gravity
  /// estimate (see ImuGravityFilter), attached to the keyframe nearest the
  /// estimate's stamp, using the data-earned sigma.
  void emit_filtered_gravity_factor_locked(const ImuGravityFilter::Estimate & est);

  /// Env-gated (MOLA_MAPPER3D_TRACE_IMU) diagnostic: logs T_enu_to_map (F0) and
  /// the distribution of IMU gravity-factor residuals (mean/median/p90/max +
  /// the mean residual VECTOR norm, which separates a systematic map tilt from
  /// random motion-acceleration noise). No-op unless the env var is set.
  void trace_imu_factors_locked(const gtsam::Values & estimate);

  /// Accumulates one raw IMU sample into imu_accum_ (max-rate summarization).
  void accumulate_imu_sample_locked(const mrpt::obs::CObservationIMU & imu);

  /// Builds a summarized CObservationIMU from imu_accum_ (averaged accel/gyro,
  /// latest orientation) and clears the accumulator. False if nothing buffered.
  [[nodiscard]] bool build_summarized_imu_locked(mrpt::obs::CObservationIMU & out);

  /// Drains pending factors/values, runs the incremental iSAM2 update and
  /// refreshes the cached estimates. Does its OWN locking in three phases
  /// (grab-pending -> heavy solve without stateMutex_ -> commit caches), so the
  /// caller must NOT hold stateMutex_ when calling it. Serialized by
  /// solve_mutex_. No-op when nothing is pending.
  void optimize_and_refresh();

  /// Background thread body: waits for pending work and calls
  /// optimize_and_refresh() (only started when enable_optimizer_thread).
  void optimizer_thread_loop();

  /// Wakes the optimizer thread after new data was enqueued (no-op when the
  /// thread is disabled).
  void notify_optimizer();

  /// Stops and joins the optimizer thread if running. Must be called with NO
  /// lock held (it joins a thread that itself takes stateMutex_).
  void stop_optimizer_thread();

  /// Writes the current estimated trajectory (all keyframe poses in
  /// {reference_frame}) to params_.save_trajectory_to_file in TUM format.
  /// No-op when that string is empty.
  void saveEstimatedTrajectoryToFile();

  /// Publishes the latest extrapolated reference-frame pose via
  /// advertiseUpdatedLocalization, throttled to high_rate_pose_publish_rate_hz.
  void publish_high_rate_pose();

  /// Returns the optimized NavState (pose+twist+covariances) of a keyframe.
  [[nodiscard]] NavState get_latest_state_and_covariance(KeyFrameID idx) const;

  /// Returns the (before, after) keyframe iterators bracketing timestamp `t`.
  [[nodiscard]] pair_nearby_frame_iterators_t find_before_after(
    const mrpt::Clock::time_point & t, bool allow_exact_match) const;

  /// Picks the temporally closest of a (before, after) pair, or nullopt.
  [[nodiscard]] std::optional<KeyFrameID> pick_closest(
    const pair_nearby_frame_iterators_t & closestFrames,
    const mrpt::Clock::time_point & stamp) const;

  /// Returns the nearest existing keyframe to `t` (no tolerance check).
  /// Returns nullopt when no keyframes exist yet.
  [[nodiscard]] std::optional<KeyFrameID> find_nearest_kf_locked(
    const mrpt::Clock::time_point & t) const;

  /// True when sensor paths (fuse_pose, fuse_imu, fuse_odometry, fuse_gnss)
  /// are allowed to create new keyframe GTSAM variables. Controlled by
  /// keyframe_creation_source and, in Auto mode, by shared_kf_producer_active_.
  [[nodiscard]] bool sensor_kf_creation_allowed() const;
};

}  // namespace mola::mapper_3d
