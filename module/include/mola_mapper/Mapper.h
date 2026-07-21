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
 * @file   Mapper.h
 * @brief  Central 3D SLAM map: online multi-sensor fusion, loop closure and
 *         geo-referencing into one optimized world model.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_imu_preintegration/ImuTransformer.h>
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_mapper/ImuGravityFilter.h>
#include <mola_mapper/Parameters.h>
#include <mola_mapper/WorldModelState.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
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

namespace mola
{
class LoopClosureInterface;
}  // namespace mola

namespace mola::mapper
{
/** Central 3D SLAM map and multi-sensor fusion backend.
 *
 * Mapper holds one global, optimized representation of the world, based on
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
 * \ingroup mola_mapper_grp
 */
class Mapper : public mola::NavStateFilter,
               public mola::LocalizationSourceBase,
               public mola::MapSourceBase,
               public mola::DiagnosticsProvider,
               public mola::SharedKeyframeMap
{
  DEFINE_MRPT_OBJECT(Mapper, mola::mapper)

public:
  Mapper();
  ~Mapper() override;

  // Non-copyable / non-movable (owns GTSAM state).
  Mapper(const Mapper &) = delete;
  Mapper & operator=(const Mapper &) = delete;
  Mapper(Mapper &&) = delete;
  Mapper & operator=(Mapper &&) = delete;

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

  /** Anchors T_enu_to_map to a known, fixed value, instead of leaving it a free
   * variable. Called by a localization front end once a geo-referenced map is
   * loaded (relocalize mode). Without this, T_enu_to_map keeps its weak
   * construction-time prior, and since every gravity/attitude factor only
   * measures rotation RELATIVE to it, the system's absolute rotation stays a
   * genuine gauge freedom that the solver cannot resolve.
   */
  void set_geo_reference(const mola::Georeferencing & georef) override;

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

  // ExecutableBase: expose the loop-closure engine's own logger (a separate
  // mrpt::system::COutputLogger instance, since it runs on its own background
  // thread) so its output gets the same console capture as the mapper itself.
  [[nodiscard]] std::vector<std::pair<ChildLoggerName, mrpt::system::COutputLogger *>>
  child_loggers() const override;

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

  // Freshest raw-observation timestamp (IMU/wheels/GNSS), stored as
  // mrpt::Clock tick count. Tracked so the high-rate publisher and the
  // camera-follow can extrapolate the {map} pose to the current data instant
  // BETWEEN the sparse keyframes: the mapper's only dense inputs in a
  // LIO+SharedKeyframeMap setup are the high-rate IMU/wheels, so the newest
  // keyframe stamp alone advances only at the keyframe cadence. Atomic +
  // monotonic (a late/out-of-order arrival never drags it backwards), mirroring
  // the smoother's last_observation_stamp. Sentinel = "nothing seen yet".
  static constexpr int64_t kNoObsStamp = std::numeric_limits<int64_t>::min();
  std::atomic<int64_t> last_observation_stamp_ticks_{kNoObsStamp};

  // --- Background loop-closure thread (loop_closure_enabled) ---
  // The LC detector (mola_sm_loop_closure) runs OFF the query/solve path: the
  // thread periodically snapshots the central map, runs the detector on the
  // snapshot (no lock held), and merges accepted edges into the graph as robust
  // BetweenFactors, then wakes the optimizer. Only the merge and snapshot touch
  // state_ (under stateMutex_); the heavy ICP runs lock-free on the snapshot.
  std::shared_ptr<mola::LoopClosureInterface> lc_engine_;
  std::thread lc_thread_;
  std::mutex lc_wakeup_mutex_;
  std::condition_variable lc_wakeup_cv_;
  std::atomic_bool lc_should_exit_{false};
  // Set by request_loop_closure_scan() (e.g. the GUI "Run LC scan now" button)
  // to make the LC thread run one immediate FULL scan instead of waiting out the
  // check period. Consumed and cleared by the thread; never runs analyze()
  // off-thread, so it cannot race the periodic scan.
  std::atomic_bool lc_force_scan_{false};

  // Progress state carried between scans by the LC thread. Reset when the thread
  // (re)starts on a fresh map.
  struct LoopClosureScanState
  {
    // Keyframe count present at the last scan (min-new-keyframes gate) and the
    // snapshot size then (the detector's incremental first_new_keyframe hint).
    std::size_t kf_count_at_last_scan = 0;
    std::size_t snapshot_size_at_last_scan = 0;
    // Incremental scans run since the last forced full scan.
    uint32_t incremental_scans_since_full = 0;
    // Keyframe pairs already merged as loop-closure edges (normalized min<max),
    // so a periodic full scan re-proposing an existing loop does not add a
    // duplicate BetweenFactor that over-weights the constraint and bloats the
    // graph.
    std::set<std::pair<KeyFrameID, KeyFrameID>> merged_pairs;

    void reset() { *this = LoopClosureScanState{}; }
  };
  LoopClosureScanState lc_scan_;

  // Live loop-closure UI counters, surfaced in the GUI "Loop Closure" tab, the
  // metric plots and getDiagnostics(). Kept as atomics (not under stateMutex_)
  // so the viz thread reads them cheaply while a heavy scan runs, and so the
  // per-candidate on_progress callback can update them without taking a lock.
  struct LoopClosureUiState
  {
    // Cumulative over the run:
    std::atomic<std::size_t> loops_accepted{0};  // == lc_scan_.merged_pairs.size()
    std::atomic<std::size_t> candidates_checked{0};
    std::atomic<std::size_t> scans_completed{0};
    // Current / most-recent scan:
    std::atomic<bool> scan_in_progress{false};
    std::atomic<std::size_t> cur_total{0};  // candidates this scan (queue size)
    std::atomic<std::size_t> cur_done{0};   // evaluated so far (pending = total - done)
    std::atomic<std::size_t> last_scan_accepted{0};
    std::atomic<double> last_scan_seconds{0.0};
    std::atomic<bool> last_scan_full{false};
    // Finalize (end-of-run batch) progress:
    std::atomic<bool> finalize_active{false};
    std::atomic<std::size_t> finalize_round{0};
    std::atomic<std::size_t> finalize_rounds_total{0};

    void reset()
    {
      loops_accepted = 0;
      candidates_checked = 0;
      scans_completed = 0;
      scan_in_progress = false;
      cur_total = 0;
      cur_done = 0;
      last_scan_accepted = 0;
      last_scan_seconds = 0.0;
      last_scan_full = false;
      finalize_active = false;
      finalize_round = 0;
      finalize_rounds_total = 0;
    }
  };
  LoopClosureUiState lc_ui_;

  /// Background worker for lazy-load externalization of keyframe point clouds
  /// in saveSimpleMapToFile(), so writing the (potentially large) point cloud
  /// files to disk does not block the caller (destructor or GUI thread).
  mutable mrpt::WorkerThreadsPool worker_disk_io_{
    1 /*num threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_disk"};

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

  // --- High-rate IMU accumulation (vehicle-frame buffer, no factors here) ---
  // fuse_imu() does NOT touch the graph: each raw reading is moved to the
  // vehicle "base_link" frame by a per-sensor ImuTransformer (rotation + rigid
  // lever-arm/centripetal correction) and pushed into ONE global
  // LocalVelocityBuffer (proper accel, angular velocity, latest absolute
  // orientation). The backend factors are built later, once per real keyframe,
  // by emit_imu_factors_for_keyframe_locked() draining the buffer window. This
  // decouples the IMU sample rate from both keyframe creation and factor
  // insertion (see agents.md "IMU gravity leveling"). Guarded by stateMutex_.
  std::map<std::string, mola::imu::ImuTransformer> imu_transformers_;
  mola::imu::LocalVelocityBuffer imu_buffer_;
  // The keyframe the last IMU window was attached to; the next keyframe drains
  // the buffer window since this one. nullopt until the first keyframe is seen.
  std::optional<KeyFrameID> last_imu_kf_;
  // Gravity-leveling factors are emitted from the IMU stream at a BOUNDED rate
  // (every imu_gravity_window_sec), NOT on keyframe creation: keyframes are
  // created from distance traveled, so they never land during the STOPS where
  // the accelerometer sees clean, motion-free gravity. last_gravity_check_t_ is
  // the timestamp [s] of the last bounded-rate check; last_gravity_kf_id_ is the
  // keyframe the last gravity factor was attached to (at most ONE per keyframe,
  // so a long stop does not over-constrain the same pose).
  std::optional<mola::imu::TimeStamp> last_gravity_check_t_;
  std::optional<KeyFrameID> last_gravity_kf_id_;
  // Reused as a STATELESS per-interval robust-gravity reducer (its accept/avg
  // math is fed the buffered window and flush()ed on each keyframe close; the
  // internal window timer is not used). Guarded by stateMutex_.
  ImuGravityFilter imu_gravity_filter_;
  // Bias linearization point for the IMU preintegration (accel xyz, gyro xyz).
  // Refreshed from the newest keyframe's optimized B(kf) after each solve so the
  // next interval preintegrates around the current best estimate. GTSAM types
  // are kept out of this public header, so it is stored as a plain array and
  // converted in Mapper_Fusion.cpp. Guarded by stateMutex_.
  std::array<double, 6> imu_bias_hat_{};

  // --- Geo-referencing diagnostics counters (guarded by stateMutex_) ---
  struct GeoRefCounters
  {
    std::size_t gnss = 0;  // factor counts
    std::size_t gnss_readings_seen = 0;
    std::size_t imu_attitude = 0;
    std::size_t imu_gravity = 0;
    std::size_t imu_omega = 0;
    std::size_t imu_preintegration = 0;
    std::size_t imu_relative_rotation = 0;
    bool georef_converged_announced = false;
  } geo_ref_counters_;

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
  // Separate wall-clock throttle for the camera-follow: it runs every spinOnce
  // (well above the scene-rebuild rate) so the camera tracks the dense vehicle
  // pose smoothly, not at the sparse keyframe / scene cadence.
  std::optional<mrpt::Clock::time_point> last_camera_follow_wallclock_;

  // Runtime view toggles, driven from the GUI "Control" tab. Written from the
  // GUI thread, read from the spinOnce/viz thread (atomic to avoid a data race;
  // a stale-by-one-frame read only delays a checkbox by one update).
  std::atomic_bool viz_show_keyframes_{true};
  std::atomic_bool viz_show_edges_{true};
  /// When true, loop-closure edges render in a distinct color/thickness so
  /// accepted loops stand out from the consecutive-keyframe odometry chain.
  std::atomic_bool viz_highlight_lc_edges_{true};
  std::atomic_bool viz_show_odom_frames_{true};
  std::atomic_bool viz_camera_follows_vehicle_{false};
  std::atomic_bool viz_show_ground_grid_{true};

  /// Keyframe XYZ-corner scale, in meters (0 = hidden). Defaults from
  /// viz_params_ ("keyframe_corner_size"); tunable at runtime from the GUI
  /// "View" tab combo box.
  std::atomic<float> viz_keyframe_corner_scale_{0.5f};
  /// Keyframe position marker (CSphere) radius, in meters (0 = hidden).
  /// Defaults from viz_params_ ("keyframe_sphere_radius"); tunable at runtime
  /// from the GUI "View" tab combo box.
  std::atomic<float> viz_keyframe_sphere_radius_{0.2f};

  /// Graph-edge (CCylinder) radius, in meters. Defaults from viz_params_
  /// ("edge_cylinder_radius"); tunable at runtime from the GUI "View" tab
  /// combo box. Edges (both consecutive-keyframe and loop-closure) render as
  /// thin cylinders instead of GL lines, which are otherwise barely visible.
  std::atomic<float> viz_edge_cylinder_radius_{0.05f};

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
    // Loop-closure tab (only populated when loop_closure_enabled).
    mola::gui::LiveString::Ptr lbLcLoops;
    mola::gui::LiveString::Ptr lbLcCandidates;
    mola::gui::LiveString::Ptr lbLcCurrent;
    mola::gui::LiveString::Ptr lbLcLast;
  };
  GuiData gui_;

#ifdef MOLA_KERNEL_VIZ_HAS_METRICS
  /// Loop-closure metric plot channels (mola_viz_imgui "Plots" menu); lazily
  /// registered once visualizer_ is available. Guarded by the feature macro so
  /// this module still builds against an older mola_kernel that predates
  /// register_metric()/push_metric().
  mola::MetricChannel::Ptr metric_lc_loops_total_;
  mola::MetricChannel::Ptr metric_lc_queue_depth_;
  mola::MetricChannel::Ptr metric_lc_candidates_per_scan_;
  mola::MetricChannel::Ptr metric_lc_scan_time_ms_;
  mola::MetricChannel::Ptr metric_lc_edge_goodness_;
  bool lc_metrics_registered_ = false;
#endif

  /// Renders the keyframe tree, graph edges and per-source movable {odom_i}
  /// frames into the visualizer, and creates/updates the GUI sub-window.
  /// Throttled to visualization.update_rate_hz. No-op when no visualizer is
  /// attached. (Mapper_GUI.cpp)
  void updateVisualization();

  /// Centers the viewport on the vehicle when "camera follows vehicle" is on.
  /// Runs every spinOnce (self-throttled) using the freshest dense fuse_pose()
  /// vehicle pose extrapolated to the current instant, so it tracks actual
  /// high-rate motion instead of jumping at the keyframe cadence. (Mapper_GUI.cpp)
  void updateCameraFollow();

  /// Builds the backend-agnostic GUI sub-window (status + control tabs) once.
  void internalBuildGUI();

  // --- helpers implemented across the Mapper_*.cpp translation units ---

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

  /// Moves one raw IMU reading to the vehicle frame (per-sensor ImuTransformer)
  /// and pushes proper accel / angular velocity / absolute orientation into the
  /// global LocalVelocityBuffer. Adds NO factor and creates NO keyframe.
  void ingest_imu_sample_locked(const mrpt::obs::CObservationIMU & imu);

  /// Builds the IMU backend factors for a freshly created keyframe by draining
  /// the LocalVelocityBuffer window since the previous IMU keyframe: ONE
  /// robust-gravity MeasuredGravityFactor (data-earned sigma), the absolute
  /// attitude Pose3RotationFactor (always, so IMU azimuth can drive geo-ref),
  /// and an averaged-gyro W prior. No-op until a previous IMU keyframe exists.
  void emit_imu_factors_for_keyframe_locked(KeyFrameID newKf);

  /// Builds and adds the between-keyframe CombinedImuFactor (the RELATIVE half of
  /// IMU fusion) by preintegrating the buffered accel+gyro over the interval
  /// (tFrom, newKf-stamp]. Assumes the Vw/B variables for both keyframes already
  /// exist (created in initialize_new_frame). Nav frame = {map}; see agents.md.
  /// No-op if imu_preintegration_enabled is false or the window has no accel.
  /// @returns true if a factor was actually added. When false, the caller MUST
  /// call add_imu_preint_fallback_priors_locked() or the new keyframe's Vw/B
  /// stay unconstrained and iSAM2 discards the batch.
  [[nodiscard]] bool emit_imu_preintegration_factor_locked(
    KeyFrameID prevKf, KeyFrameID newKf, mola::imu::TimeStamp tFrom,
    const mola::imu::LocalVelocityBuffer::SamplesByTime & window);

  /// Shared coverage check for the IMU integrators: true if `integratedTime`
  /// spans enough of the keyframe interval for the delta to be meaningful.
  /// Logs a throttled warning (tagged `what`) when it does not.
  [[nodiscard]] bool has_sufficient_interval_coverage_locked(
    const char * what, KeyFrameID newKf, mola::imu::TimeStamp tFrom, double integratedTime) const;

  /// Loose priors on Vw(kf)/B(kf) for when no IMU factor could be built for this
  /// keyframe (empty/undercovered window). Keeps the graph well-posed through
  /// IMU dropouts instead of letting iSAM2 throw and drop the batch.
  void add_imu_preint_fallback_priors_locked(KeyFrameID kf);

  /// Builds and adds the lightweight gyro RELATIVE-rotation factor between two
  /// keyframes (Pose3RelativeRotationFactor): integrates the interval gyro into
  /// deltaRij and constrains R_i^{-1} R_j to it. No position/velocity integral,
  /// no gravity: robust over long keyframe intervals. No-op if
  /// imu_relative_rotation_enabled is false or the window has no gyro samples.
  void emit_imu_relative_rotation_factor_locked(
    KeyFrameID prevKf, KeyFrameID newKf, mola::imu::TimeStamp tFrom,
    const mola::imu::LocalVelocityBuffer::SamplesByTime & window);

  /// Bounded-rate gravity-leveling emission, driven from the IMU stream (NOT
  /// keyframe creation). Every imu_gravity_window_sec it reduces the recent
  /// proper-acceleration window and, if it is a clean low-dynamics reading,
  /// attaches ONE MeasuredGravityFactor to the latest keyframe. This is what
  /// lets the clean gravity seen during STOPS (when no keyframe is created) level
  /// the map. Called from ingest_imu_sample_locked under stateMutex_.
  void maybe_emit_gravity_factor_locked(mola::imu::TimeStamp tNow);

  /// Env-gated (MOLA_MAPPER_TRACE_IMU) diagnostic: logs T_enu_to_map (F0) and
  /// the distribution of IMU gravity-factor residuals (mean/median/p90/max +
  /// the mean residual VECTOR norm, which separates a systematic map tilt from
  /// random motion-acceleration noise). No-op unless the env var is set.
  void trace_imu_factors_locked(const gtsam::Values & estimate);

  /// Env-gated (MOLA_MAPPER_TRACE_GEOM) diagnostic: measures the ACTUAL
  /// keyframe trajectory geometry in {map} (not the gravity-factor residual):
  /// the keyframe-position z-span and the best-fit slope of z vs horizontal
  /// arc length (the apparent path tilt), plus the average keyframe body up
  /// axis vs map +Z (orientation leveling). This separates a real {map}-frame
  /// path tilt from an uncorrected {odom} LIO-local-map tilt. No-op unless set.
  void trace_keyframe_geometry_locked(const gtsam::Values & estimate);

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

  /// Creates the loop-closure engine from loop_closure_pipeline_file and starts
  /// the background LC thread. No-op when loop_closure_enabled is false.
  /// (Mapper_LoopClosure.cpp)
  void start_loop_closure_thread();

  /// Background loop-closure thread body: waits for the check period, then runs
  /// one scan. Only started when loop_closure_enabled.
  void loop_closure_thread_loop();

  /// Stops and joins the loop-closure thread if running (no lock held).
  void stop_loop_closure_thread();

  /// Requests one immediate FULL scan by the background LC thread (wakes it
  /// early). Thread-safe, non-blocking; a no-op when the LC thread is not
  /// running. Used by the GUI "Run LC scan now" button.
  void request_loop_closure_scan();

  /// Shared shutdown sequence for the loop-closure thread: signals exit,
  /// wakes it, and joins it if running. Resets lc_engine_ only when
  /// resetEngine is true (finalize_loop_closures() keeps the engine alive for
  /// its synchronous batch rounds; stop_loop_closure_thread() does not).
  void stop_loop_closure_thread_locked(bool resetEngine);

  /// Runs one loop-closure scan: snapshots the map under stateMutex_, runs the
  /// detector off-lock (streaming + abortable), merges accepted edges, and wakes
  /// the optimizer. Returns the number of edges merged. When forceFullScan is
  /// true, the min-new-keyframes gate and incremental hint are bypassed so the
  /// entire map is re-examined (used by the finalize pass).
  std::size_t run_loop_closure_scan(bool forceFullScan = false);

  /// Lazily registers the loop-closure metric plot channels once visualizer_ is
  /// available (no-op afterwards, or when built against an older mola_kernel
  /// without the metrics API). Called at the start of each scan.
  void register_lc_metrics_if_needed();

  /// After the dataset ends, repeatedly runs full loop-closure scans interleaved
  /// with a synchronous re-optimization, until a round finds no new loops or the
  /// round budget is exhausted. Online scans only close loops near the end of the
  /// run (both endpoints must exist and drift must already be small); this batch
  /// pass recovers the remaining loops now that the whole trajectory is present
  /// and each round's optimization shrinks the drift for the next. No-op when
  /// loop_closure_finalize_rounds is 0. (Mapper_LoopClosure.cpp)
  void finalize_loop_closures();

  /// Adds one accepted loop-closure edge as a robust BetweenFactor(T(from),
  /// T(to)) built from the proposed relative pose + covariance. Returns false
  /// (adding nothing) if this pair was already closed in an earlier scan.
  /// Caller holds stateMutex_.
  bool merge_loop_closure_edge_locked(
    KeyFrameID from, KeyFrameID to, const mrpt::poses::CPose3DPDFGaussian & relPose);

  /// Writes the current estimated trajectory (all keyframe poses in
  /// {reference_frame}) to params_.save_trajectory_to_file in TUM format.
  /// No-op when that string is empty.
  void saveEstimatedTrajectoryToFile();

  /// Writes the current keyframes (raw observations + optimized poses) as a
  /// CSimpleMap to params_.save_simplemap_file. No-op when that string is
  /// empty.
  void saveSimpleMapToFile();

  /// Publishes the latest extrapolated reference-frame pose via
  /// advertiseUpdatedLocalization, throttled to high_rate_pose_publish_rate_hz.
  void publish_high_rate_pose();

  /// Freshest instant the {map}-frame pose can be extrapolated to: the newest
  /// raw source anchor, falling back to the newest keyframe. This advances
  /// between keyframes as dense fuse_pose() readings arrive, so both the
  /// high-rate publisher and the camera-follow track actual vehicle motion
  /// rather than the sparse keyframe cadence. Mirrors the smoother's
  /// get_current_extrapolated_stamp(). Caller must hold stateMutex_.
  [[nodiscard]] std::optional<mrpt::Clock::time_point> get_current_extrapolated_stamp_locked()
    const;

  /// Records the freshest raw-observation timestamp into
  /// last_observation_stamp_ticks_ (monotonic; older stamps are ignored).
  /// Lock-free; safe to call from any sensor callback.
  void note_observation_stamp(const mrpt::Clock::time_point & t);

  /// Returns the optimized NavState (pose+twist+covariances) of a keyframe.
  /// Throws if `idx` has no solved estimate yet; callers must handle that.
  [[nodiscard]] NavState get_latest_state_and_covariance(KeyFrameID idx) const;

  /// Body of estimated_navstate(). Kept separate so the public entry point can
  /// turn any unexpected failure into the "not ready yet" nullopt its signature
  /// already promises, instead of letting it escape into the caller's thread.
  [[nodiscard]] std::optional<NavState> estimated_navstate_impl(
    const mrpt::Clock::time_point & timestamp, const std::string & frame_id);

  /// Freshest dense fuse_pose() vehicle pose expressed in {map}, extrapolated to
  /// `atStamp` with the source's own (filtered) twist. Unlike estimated_navstate
  /// it is GATE-FREE: it does not require the nearest keyframe to be solved or
  /// within the velocity-model window, so it stays smooth for the camera even
  /// when the newest keyframe is unsolved or sparse. Returns nullopt when no raw
  /// source anchor with a resolvable T_map_to_odom exists yet. (Mapper_Fusion.cpp)
  [[nodiscard]] std::optional<mrpt::poses::CPose3D> freshest_vehicle_pose_in_map(
    const mrpt::Clock::time_point & atStamp) const;

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

}  // namespace mola::mapper
