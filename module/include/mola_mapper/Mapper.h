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
#include <mola_mapper/GlobalGraph.h>
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
}

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
 *  - publishes the estimated geo-reference (`mola::MapSourceBase`) and writes the
 *    optimized map, trajectory and geo-reference at the end.
 *
 *  The map frame is the one of the front end keyframes: the first keyframe stays
 *  at its front end pose, which is also where the state estimator starts.
 *
 * \ingroup mola_mapper_grp
 */
class Mapper : public mola::ExecutableBase,
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
  void onNewObservation(const CObservation::ConstPtr & o) override;
  std::optional<KeyFrameID> requestInsertKeyframe(const KeyframeInsertRequest & req) override;
  void getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status) override;
  /** @} */

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
  };

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

  /** @name State, under state_mtx_
     * @{ */
  mutable std::mutex state_mtx_;
  std::vector<Keyframe> keyframes_;
  std::deque<mrpt::obs::CObservationGPS::Ptr> pending_gnss_;
  std::deque<ImuSample> imu_samples_;
  mrpt::obs::CObservationIMU::Ptr last_imu_;
  std::vector<GlobalGraphLoopEdge> loops_;
  std::set<std::pair<std::size_t, std::size_t>> loop_pairs_;
  std::optional<mrpt::topography::TGeodeticCoords> geodetic_datum_;
  std::optional<mola::Georeferencing> georef_;
  GlobalGraphResult last_result_;
  /// Keyframes and loops already included in the last solve.
  std::size_t solved_keyframes_ = 0;
  std::size_t solved_loops_ = 0;
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
  /// Returns false if there was nothing new to solve.
  bool solve_global_graph();
  void send_feedback(const Keyframe & kf, const GlobalGraphResult & r);
  std::weak_ptr<mola::NavStateFilter> state_estimator_;
  /** @} */

  /** @name Loop closure
     * @{ */
  std::shared_ptr<mola::LoopClosureInterface> lc_engine_;
  std::thread lc_thread_;
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
  std::size_t lc_kf_count_at_last_scan_ = 0;       // LC thread only
  std::size_t lc_snapshot_size_at_last_scan_ = 0;  // LC thread only
  std::size_t lc_incremental_scans_ = 0;           // LC thread only

  void start_loop_closure_thread();
  void stop_loop_closure_thread();
  void loop_closure_thread_loop();
  std::size_t run_loop_closure_scan(bool forceFullScan);
  /** @} */

  void publish_georeference(const mola::Georeferencing & g);
  void save_outputs();
};

}  // namespace mola::mapper
