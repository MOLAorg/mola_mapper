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
#include <mola_mapper/LoopOdometryCheck.h>
#include <mola_mapper/Parameters.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>

#include <atomic>
#include <condition_variable>
#include <deque>
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

  [[nodiscard]] std::optional<mola::Georeferencing> georeferencing() const;

  /// The central map: optimized keyframe poses with all their observations.
  [[nodiscard]] mrpt::maps::CSimpleMap simple_map() const;

  /// Solves the global graph now, in the calling thread.
  void optimize_now();

  /// Wakes the loop closure thread for one full scan (returns immediately).
  void request_loop_closure_scan();

  /** Blocks until the loop closure thread has no scan running nor requested.
     *  Returns true at once if there is no loop closure thread. A timeout <= 0
     *  waits forever. */
  bool wait_for_loop_closure_idle(double timeoutSeconds = 0);

  /// Runs one loop closure scan in the calling thread. Returns the new edges.
  std::size_t run_loop_closure_scan_now(bool forceFullScan = true);

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
    /// GNSS readings and gravity attached by the mapper.
    mrpt::obs::CSensoryFrame::Ptr aux = mrpt::obs::CSensoryFrame::Create();
    /// Time offset of the attached GNSS reading [s] (to keep the nearest one).
    std::optional<double> gnss_time_offset;
    /// Current estimate in the map frame.
    mrpt::poses::CPose3D pose_in_map;
    /// Gyroscope rotation (vehicle frame) since the previous keyframe, sigma [rad].
    std::optional<mrpt::poses::CPose3D> gyro_rotation_from_prev;
    double gyro_rotation_sigma = 0;
  };

  /// Gyroscope bias (and its sigma [rad/s]) at time t, from the bias sources.
  struct GyroBias
  {
    mrpt::math::TVector3D bias{0, 0, 0};
    double sigma = 0;
  };
  /// Queries the bias sources: call without state_mtx_ held.
  [[nodiscard]] GyroBias query_gyro_bias(
    const std::string & label, const mrpt::Clock::time_point & t) const;

  /// Gyroscope rotation from the previous keyframe to kf (state_mtx_ held).
  void attach_gyro_rotation_locked(Keyframe & kf, const GyroBias & b);

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
  std::vector<std::shared_ptr<mola::ImuBiasSource>> imu_bias_sources_;

  /** @name State, under state_mtx_
     * @{ */
  mutable std::mutex state_mtx_;
  std::vector<Keyframe> keyframes_;
  std::deque<mrpt::obs::CObservationGPS::Ptr> pending_gnss_;
  std::deque<ImuSample> imu_samples_;
  /// Accelerometer readings of the first IMU matching imu_sensor_labels_re.
  std::string gravity_label_;
  mrpt::poses::CPose3D gravity_sensor_pose_;
  /// Gyroscope readings of the first IMU matching gyro_sensor_labels_re.
  std::deque<GyroSample> gyro_samples_;
  std::string gyro_label_;
  mrpt::poses::CPose3D gyro_sensor_pose_;
  std::vector<GlobalGraphLoopEdge> loops_;
  /// Keyframe pairs (min, max) of the merged loops.
  std::set<std::pair<std::size_t, std::size_t>> loop_pairs_;
  /// Keyframe pairs (min, max) of the loops rejected by the odometry check.
  std::set<std::pair<std::size_t, std::size_t>> rejected_loop_pairs_;
  std::optional<mrpt::topography::TGeodeticCoords> geodetic_datum_;
  std::optional<mola::Georeferencing> georef_;
  /// Factor counts of the last solve, for diagnostics.
  std::size_t last_num_gnss_factors_ = 0;
  std::size_t last_num_gravity_factors_ = 0;
  std::size_t last_num_gyro_factors_ = 0;
  /// Loops in the last solve advertised as a local map correction (solve_mtx_).
  std::size_t loops_in_last_correction_ = 0;
  /// Stamp of the last keyframe pose fed back to the state estimator.
  std::optional<mrpt::Clock::time_point> last_feedback_stamp_;
  bool finalized_ = false;
  /** @} */

  void attach_gnss_locked(const mrpt::obs::CObservationGPS::Ptr & gps);
  void retry_pending_gnss_locked();
  void attach_gravity_locked(Keyframe & kf);

  /** @name Global graph optimizer
     * @{ */
  std::mutex solve_mtx_;  ///< one solve at a time
  std::thread optimizer_thread_;
  std::mutex optimizer_wakeup_mtx_;
  std::condition_variable optimizer_wakeup_cv_;
  std::atomic_bool optimizer_should_exit_{false};
  std::atomic_bool optimizer_new_data_{false};
  std::atomic<std::size_t> solves_{0};

  void optimizer_thread_loop();
  void notify_optimizer();
  void stop_optimizer_thread();
  void solve_global_graph();
  [[nodiscard]] GlobalGraphInput graph_input_locked() const;
  void send_feedback(
    const mrpt::Clock::time_point & stamp, const mrpt::poses::CPose3D & pose,
    const GlobalGraphResult & r);
  std::weak_ptr<mola::NavStateFilter> state_estimator_;
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
  /** @} */

  void publish_georeference(const mola::Georeferencing & g);
  void save_outputs();
  void save_state_estimator_outputs(const std::vector<Keyframe> & kfs);
  [[nodiscard]] static mrpt::maps::CSimpleMap to_simple_map(const std::vector<Keyframe> & kfs);
};

}  // namespace mola::mapper
