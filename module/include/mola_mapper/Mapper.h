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
 * @brief  Central keyframe map with loop closure and a global pose graph.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#if __has_include(<mola_kernel/interfaces/LocalMapCorrectionSource.h>)
#include <mola_kernel/interfaces/LocalMapCorrectionSource.h>
#define MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS 1
#endif
#include <mola_mapper/GlobalGraph.h>
#include <mola_mapper/GyroIntegration.h>
#include <mola_mapper/LoopConsistency.h>
#include <mola_mapper/LoopOdometryCheck.h>
#include <mola_mapper/MapperInputLog.h>
#include <mola_mapper/Parameters.h>
#include <mola_mapper/WheelOdometry.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
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

namespace mola
{
class LoopClosureInterface;
struct ProposedLoopEdge;
class ImuBiasSource;
class VizInterface;
}  // namespace mola

namespace mola::mapper
{
/** Central keyframe map of a MOLA system: the global back end next to a front end
 *  (LIO, VO) and its state estimator (mola_state_estimation_smoother).
 *
 *  It is not a state estimator itself. It runs as a separate module that:
 *  - receives the keyframes of the front end (`mola::SharedKeyframeMap`) and
 *    attaches to each one the GNSS readings and the averaged accelerometer
 *    (gravity) of the dataset (`mola::RawDataConsumer`);
 *  - looks for loop closures between them in a background thread, with the
 *    detector of mola_sm_loop_closure;
 *  - optimizes a pose graph of all the keyframes (odometry chain, loop closures,
 *    GNSS and gravity; see GlobalGraph.h) in another background thread;
 *  - optionally feeds the optimized pose of the newest keyframe back to the state
 *    estimator, as an absolute pose in the map frame, so loop closures and GNSS
 *    correct the live estimate;
 *  - after loop closures, tells the front ends (`mola::LocalMapCorrectionSource`)
 *    and gives them the corrected keyframes around them, to rebuild their local
 *    maps;
 *  - publishes the estimated geo-reference (`mola::MapSourceBase`) and writes the
 *    optimized map, trajectory and geo-reference at the end.
 *
 *  The map frame is the one of the front end keyframes: the first keyframe stays
 *  at its front end pose, which is also where the state estimator starts.
 *
 * \ingroup mola_mapper_grp
 */
class Mapper : public mola::ExecutableBase,
#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
               public mola::LocalMapCorrectionSource,
#endif
               public mola::RawDataConsumer,
               public mola::MapSourceBase,
               public mola::DiagnosticsProvider,
               public mola::SharedKeyframeMap
{
  DEFINE_MRPT_OBJECT(Mapper, mola::mapper)

public:
  Mapper();
  ~Mapper() override;

  Mapper(const Mapper &) = delete;
  Mapper & operator=(const Mapper &) = delete;
  Mapper(Mapper &&) = delete;
  Mapper & operator=(Mapper &&) = delete;

  /** @name Module API
     * @{ */
  void initialize(const mrpt::containers::yaml & cfg) override;
  void spinOnce() override;
  /// Runs finalize() (the other modules still exist at this point).
  void onQuit() override;
  void onNewObservation(const CObservation::ConstPtr & o) override;
  std::optional<KeyFrameID> requestInsertKeyframe(const KeyframeInsertRequest & req) override;
  void getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status) override;
  /** @} */

#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
private:
  /// Keyframes within radius of keyframe `reference` (its index), oldest first.
  [[nodiscard]] std::optional<std::vector<mola::LocalMapKeyframe>> local_map_keyframes_impl(
    KeyFrameID reference, double radius) const override;

public:
#endif

  /** @name Queries and synchronous control (offline runs, tests)
     * @{ */
  [[nodiscard]] const Parameters & parameters() const { return params_; }

  [[nodiscard]] std::size_t keyframe_count() const;

  /// Current estimate of each keyframe pose in the map frame.
  [[nodiscard]] std::vector<mrpt::poses::CPose3D> keyframe_poses() const;

  [[nodiscard]] std::size_t loop_closure_count() const;

  /// Keyframe pairs (min, max) of the loop closures in the graph, in order.
  [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> loop_closure_pairs() const;

  /// Whether there is new data (keyframes, loop closures) since the last solve.
  [[nodiscard]] bool solve_pending() const { return optimizer_new_data_.load(); }

  [[nodiscard]] std::optional<mola::Georeferencing> georeferencing() const;

  /// The central map: optimized keyframe poses with all their observations.
  [[nodiscard]] mrpt::maps::CSimpleMap simple_map() const;

  /// Solves the global graph now, in the calling thread.
  void optimize_now();

  /** Inserts a keyframe with the given gyroscope bias, instead of the one of a
   *  bias source (requestInsertKeyframe() queries it): to replay recorded runs.
   *  A bias not from a source is replaced by the configured gyro_bias_sigma. */
  std::optional<KeyFrameID> insert_keyframe(
    const KeyframeInsertRequest & req, const RecordedGyroBias & gyroBias);

  /// Wakes the loop closure thread for one full scan (returns immediately).
  void request_loop_closure_scan();

  /** Blocks until the loop closure thread has no scan running nor requested.
     *  Returns true at once if there is no loop closure thread. A timeout <= 0
     *  waits forever. */
  bool wait_for_loop_closure_idle(double timeoutSeconds = 0);

  /// Runs one loop closure scan in the calling thread. Returns the new edges.
  std::size_t run_loop_closure_scan_now(bool forceFullScan = true);

  /** Adds loop closures found elsewhere (e.g. given by hand), through the same
   *  checks as the detected ones. Returns how many joined the graph (new ones,
   *  and older ones accepted again by the consistency check). */
  std::size_t add_loop_closures(const std::vector<mola::ProposedLoopEdge> & edges);

  /** End of run: loop closure passes (full scan + re-optimization) until no new
     *  loop is found, then the final solve and the outputs. Called by the
     *  destructor if not called before. */
  void finalize();
  /** @} */

private:
  struct Keyframe
  {
    mrpt::Clock::time_point stamp;
    std::string source_frame;
    mrpt::poses::CPose3DPDFGaussian pose_in_source;
    /// Observations of the front end (e.g. the LiDAR scan).
    mrpt::obs::CSensoryFrame observations;
    /// GNSS readings, images and gravity attached by the mapper.
    mrpt::obs::CSensoryFrame::Ptr aux = mrpt::obs::CSensoryFrame::Create();
    /// Observations attached as the nearest one in time, per slot ("gnss", or
    /// "image:<label>"), with their time offset [s], to keep the nearest one.
    struct Nearest
    {
      mrpt::obs::CObservation::Ptr obs;
      double dt = 0;
    };
    std::map<std::string, Nearest> nearest;
    /// Current estimate in the map frame.
    mrpt::poses::CPose3D pose_in_map;
    /// Gyroscope rotation (vehicle frame) since the previous keyframe, sigma [rad].
    std::optional<mrpt::poses::CPose3D> gyro_rotation_from_prev;
    double gyro_rotation_sigma = 0;
    /// Wheel odometry increment (vehicle frame) since the previous keyframe.
    std::optional<mrpt::math::TPose2D> wheel_odom_from_prev;
  };

  /// Gyroscope bias (and its sigma [rad/s]) at time t, from the bias sources.
  struct GyroBias
  {
    mrpt::math::TVector3D bias{0, 0, 0};
    double sigma = 0;
    bool from_source = false;
  };
  /// Queries the bias sources: call without state_mtx_ held.
  [[nodiscard]] GyroBias query_gyro_bias(
    const std::string & label, const mrpt::Clock::time_point & t) const;

  /// Gyroscope rotation from the previous keyframe to kf (state_mtx_ held).
  void attach_gyro_rotation_locked(Keyframe & kf, const GyroBias & b);
  /// Wheel odometry increment from the previous keyframe to keyframe `idx`.
  /// Returns false if the readings do not reach it yet.
  bool attach_wheel_odom_locked(std::size_t idx);
  void add_wheel_odom_sample_locked(const mrpt::obs::CObservationOdometry & odo);

  struct ImuSample
  {
    mrpt::Clock::time_point stamp;
    mrpt::math::TVector3D acc;
  };

  Parameters params_;
  std::regex gnss_labels_re_;
  std::regex imu_labels_re_;
  bool use_gnss_input_ = false;
  bool use_imu_input_ = false;
  std::regex gyro_labels_re_;
  bool use_gyro_input_ = false;
  std::regex wheel_odom_labels_re_;
  bool use_wheel_odom_input_ = false;
  std::regex image_labels_re_;
  bool use_image_input_ = false;
  std::vector<std::shared_ptr<mola::ImuBiasSource>> imu_bias_sources_;

  /** @name State, under state_mtx_
     * @{ */
  mutable std::mutex state_mtx_;
  std::vector<Keyframe> keyframes_;
  std::deque<mrpt::obs::CObservationGPS::Ptr> pending_gnss_;
  /// Images newer than the newest keyframe, waiting for the next one.
  std::deque<mrpt::obs::CObservationImage::Ptr> pending_images_;
  /// Attached images not written to a file yet: (keyframe index, slot). A
  /// closer image may still replace them.
  std::vector<std::pair<std::size_t, std::string>> unwritten_images_;
  std::deque<ImuSample> imu_samples_;
  /// Accelerometer readings of the first IMU matching imu_sensor_labels_re.
  std::string gravity_label_;
  mrpt::poses::CPose3D gravity_sensor_pose_;
  /// Gyroscope readings of the first IMU matching gyro_sensor_labels_re.
  std::deque<GyroSample> gyro_samples_;
  std::string gyro_label_;
  mrpt::poses::CPose3D gyro_sensor_pose_;
  /// Wheel odometry readings of the first label matching wheel_odom_sensor_labels_re.
  std::deque<WheelOdomSample> wheel_odom_samples_;
  std::string wheel_odom_label_;
  /// Keyframes inserted before the wheel odometry readings reached them.
  std::deque<std::size_t> pending_wheel_odom_;
  std::vector<GlobalGraphLoopEdge> loops_;
  /// Keyframe pairs (min, max) of every loop that passed the odometry check,
  /// in the graph or not (not proposed again).
  std::set<std::pair<std::size_t, std::size_t>> loop_pairs_;
  /// Keyframe pairs (min, max) of the loops rejected by the odometry check.
  std::set<std::pair<std::size_t, std::size_t>> rejected_loop_pairs_;
  /// Keyframe pairs of the loops rejected by the consistency check right now.
  std::vector<std::pair<std::size_t, std::size_t>> consistency_rejected_pairs_;
  std::optional<mrpt::topography::TGeodeticCoords> geodetic_datum_;
  std::optional<mola::Georeferencing> georef_;
  /// Factor counts of the last solve, for diagnostics.
  std::size_t last_num_gnss_factors_ = 0;
  std::size_t last_num_gravity_factors_ = 0;
  std::size_t last_num_gyro_factors_ = 0;
  std::size_t last_num_wheel_odom_factors_ = 0;
  /// Changes of loops_ (additions and removals), and loops ever added to it:
  std::size_t loops_revision_ = 0;
  std::size_t loops_added_ = 0;
  /// The same, at the last solve advertised as a local map correction (solve_mtx_).
  std::size_t loops_revision_in_last_correction_ = 0;
  std::size_t loops_added_in_last_correction_ = 0;
  /// Distance along its front end chain of each keyframe, for the loop
  /// consistency check, and the newest keyframe of each chain:
  std::vector<double> keyframe_path_;
  std::map<std::string, std::size_t> chain_last_keyframe_;
  std::map<std::string, std::size_t> chain_ids_;
  bool finalized_ = false;
  /** @} */

  enum class AttachResult
  {
    Attached,
    Pending,  ///< no keyframe at or after it yet
    Dropped
  };
  /** Attaches `o` to the keyframe nearest in time, within maxOffset, unless
   *  that one already has a closer observation in the same slot. Only once a
   *  keyframe at or after it exists: from then on, its two neighbors in time
   *  are final. */
  AttachResult attach_nearest_locked(
    const mrpt::obs::CObservation::Ptr & o, const std::string & slot, double maxOffset,
    std::size_t * attachedTo = nullptr);
  void attach_gnss_locked(const mrpt::obs::CObservationGPS::Ptr & gps);
  void retry_pending_gnss_locked();

  /// Attaches the image, or keeps it pending (returns false if pending).
  bool attach_or_pend_image_locked(const mrpt::obs::CObservationImage::Ptr & img);
  void retry_pending_images_locked();
  /** Replaces attached images that no closer image can replace any more (all,
   *  if `newest` is empty) by copies referring to image files, written by
   *  disk_io_. `newest`: label and time of the newest image. */
  void finish_images_locked(
    const std::optional<std::pair<std::string, mrpt::Clock::time_point>> & newest);
  /// Directory of the image files (empty: images are kept inline).
  std::string image_dir_;
  /// Writes the image files, so encoding does not delay the data sources.
  mrpt::WorkerThreadsPool disk_io_{1, mrpt::WorkerThreadsPool::POLICY_FIFO, "mapper_disk"};
  /// Image writes not done yet, waited for before saving the outputs (state_mtx_).
  std::deque<std::future<void>> image_writes_;
  std::optional<KeyFrameID> insert_keyframe_impl(
    const KeyframeInsertRequest & req, const RecordedGyroBias & bias,
    const mrpt::serialization::CSerializable::Ptr & recordEntry);
  void add_gyro_sample_locked(const mrpt::obs::CObservationIMU & imu);
  void add_gravity_sample_locked(const mrpt::obs::CObservationIMU & imu);
  /// Returns false if the IMU readings before the keyframe are not enough.
  bool attach_gravity_locked(Keyframe & kf);
  /// See Parameters::record_input.
  std::unique_ptr<MapperInputRecorder> recorder_;
  /// Keyframes inserted before the IMU readings of their window arrived.
  std::deque<std::size_t> pending_gravity_;

  /** @name Global graph optimizer
     * @{ */
  std::mutex solve_mtx_;  ///< one solve at a time
  std::thread optimizer_thread_;
  std::mutex optimizer_wakeup_mtx_;
  std::condition_variable optimizer_wakeup_cv_;
  std::atomic_bool optimizer_should_exit_{false};
  std::atomic_bool optimizer_new_data_{false};
  std::atomic<std::size_t> solves_{0};
  std::atomic<double> last_solve_seconds_{0};

  void optimizer_thread_loop();
  void notify_optimizer();
  void stop_optimizer_thread();
  void solve_global_graph();
  [[nodiscard]] GlobalGraphInput graph_input_locked() const;
  /** @} */

  /** @name Loop closure
     * @{ */
  std::shared_ptr<mola::LoopClosureInterface> lc_engine_;
  /// One scan at a time (the thread, run_loop_closure_scan_now(), finalize()).
  std::mutex lc_scan_mtx_;
  std::thread lc_thread_;
  /// True while lc_thread_ runs its loop (set before it starts, cleared on exit
  /// under lc_idle_mtx_), so waiters never read lc_thread_ itself.
  std::atomic_bool lc_thread_running_{false};
  std::mutex lc_wakeup_mtx_;
  std::condition_variable lc_wakeup_cv_;
  std::atomic_bool lc_should_exit_{false};
  std::atomic_bool lc_force_scan_{false};
  /// Set while a scan runs, before lc_force_scan_ is cleared: a waiter sampling
  /// both never sees "idle" in between.
  std::atomic_bool lc_busy_{false};
  std::mutex lc_idle_mtx_;
  std::condition_variable lc_idle_cv_;
  std::atomic<std::size_t> lc_scans_{0};
  std::atomic<std::size_t> lc_candidates_checked_{0};
  std::atomic<std::size_t> lc_rejected_by_odometry_{0};
  /// Loops rejected by the consistency check right now (it can decrease):
  std::atomic<std::size_t> lc_currently_rejected_by_consistency_{0};
  std::size_t lc_kf_count_at_last_scan_ = 0;       // under lc_scan_mtx_
  std::size_t lc_snapshot_size_at_last_scan_ = 0;  // under lc_scan_mtx_
  std::size_t lc_incremental_scans_ = 0;           // under lc_scan_mtx_

  void start_loop_closure_thread();
  void stop_loop_closure_thread();
  void loop_closure_thread_loop();
  std::size_t run_loop_closure_scan(bool forceFullScan);
  /// Checks a loop between keyframes first < last against the odometry chain;
  /// nothing if they are not all from the same front end chain (state_mtx_ held).
  [[nodiscard]] std::optional<LoopOdometryCheck> check_against_odometry_locked(
    std::size_t first, std::size_t last, const mola::ProposedLoopEdge & e) const;
  /// Odometry check, consistency check and merge of new loop edges
  /// (lc_scan_mtx_ held). Returns how many joined the graph.
  std::size_t merge_loop_edges(const std::vector<mola::ProposedLoopEdge> & edges);
  /// The loop with the front end odometry of its keyframes (state_mtx_ held).
  [[nodiscard]] LoopForConsistency loop_for_consistency_locked(const GlobalGraphLoopEdge & e);
  /// All loops that passed the odometry check, and the ones accepted
  /// (lc_scan_mtx_): loops_ is the accepted ones, in this order.
  std::optional<LoopConsistencyGate> lc_gate_;
  std::vector<GlobalGraphLoopEdge> lc_gate_edges_;
  /** @} */

  /** @name GUI (only with a visualizer module), see Mapper_GUI.cpp
     * @{ */
  std::shared_ptr<mola::VizInterface> visualizer_;
  struct Gui;
  std::shared_ptr<Gui> gui_;
  void gui_initialize();
  /// Redraws the graph if it changed (spinOnce() thread).
  void gui_update();
  void gui_build_window();
  /** @} */

  void publish_georeference(const mola::Georeferencing & g);
  void save_outputs();
  void save_state_estimator_outputs(const std::vector<Keyframe> & kfs);
  [[nodiscard]] static mrpt::maps::CSimpleMap to_simple_map(const std::vector<Keyframe> & kfs);
};

}  // namespace mola::mapper
