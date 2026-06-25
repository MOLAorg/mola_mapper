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

#include <mola_kernel/interfaces/DiagnosticsProvider.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mola_mapper_3d/Parameters.h>
#include <mola_mapper_3d/WorldModelState.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <map>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <string>
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

  // Wheel-odometry integration anchor (last absolute reading per single source).
  std::optional<mrpt::poses::CPose2D> last_wheels_odometry_;
  std::optional<std::string> last_wheels_odometry_name_;
  // Stamps of the last *processed* (not decimated) high-rate readings.
  std::optional<mrpt::Clock::time_point> last_wheels_odometry_stamp_;
  std::optional<mrpt::Clock::time_point> last_processed_imu_stamp_;

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

  /// Runs the incremental iSAM2 update and refreshes the cached estimates.
  void process_pending_gtsam_updates_locked();

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
