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

  // --- Geo-referencing diagnostics counters (guarded by stateMutex_) ---
  std::size_t gnss_factors_inserted_ = 0;
  std::size_t imu_factors_inserted_ = 0;
  bool georef_converged_announced_ = false;

  // Wheel-odometry relative-chaining aggregation (aggregate_high_rate_into_edges):
  // the keyframe the wheel chain last attached to, and the absolute wheel
  // odometry reading at that keyframe. On a keyframe transition, ONE relative
  // edge Between(T(last_kf), T(new_kf)) is emitted using the net wheel motion
  // since the anchor (no per-sample keyframe / absolute factor).
  std::optional<KeyFrameID> wheel_chain_last_kf_;
  std::optional<mrpt::poses::CPose2D> wheel_chain_anchor_odom_;

  // Per-source bookkeeping for keyframe-insertion requests (SharedKeyframeMap
  // sink): the last keyframe id and the front end's own pose_in_source mean it
  // was inserted with, used to chain consecutive requests via their
  // *relative* motion (see 2.8 drift-fix design /
  // request_insert_keyframe_locked()). The request's covariance is NOT used:
  // the chain/anchor factors always use our own configured
  // keyframe_ingestion_sigma_* noise, not the front end's self-reported
  // uncertainty (which can be pathologically tiny, e.g. a relocalization seed).
  struct KeyframeIngestionSourceState
  {
    KeyFrameID last_kf_id = 0;
    mrpt::poses::CPose3D last_pose_in_source;
  };
  std::map<std::string, KeyframeIngestionSourceState> keyframe_ingestion_state_by_source_;

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
};

}  // namespace mola::mapper_3d
