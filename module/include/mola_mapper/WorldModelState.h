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
 * @file   WorldModelState.h
 * @brief  Central, optimized world model: keyframes + factor graph + geo-ref.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mrpt/containers/bimap.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/topography/data_types.h>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace mola::mapper
{
/// Stable, monotonic keyframe identifier.
using KeyFrameID = mrpt::graphs::TNodeID;

/// Numeric identifier for an odometry source frame (e.g. "odom_lidar").
using OdometryFrameID = uint32_t;

struct DummyEdgeData
{
};

/// Topological connectivity between keyframes (undirected use via both edges).
using KeyFrameConnectivity = mrpt::graphs::CDirectedGraph<DummyEdgeData>;
using KeyFrameConnectivityDijkstra = mrpt::graphs::CDijkstra<KeyFrameConnectivity>;

/** Holds the full state of the central world model: the GTSAM factor graph,
 *  the keyframe raw observations (as a CSimpleMap-like store), the odometry
 *  frame registry, keyframe connectivity, and the latest optimized poses.
 *
 *  GTSAM types are hidden behind a pimpl so the public Mapper header does not
 *  pull in GTSAM (faster builds, smaller include surface).
 *
 *  Thread-safety: this class is NOT internally synchronized; the owning
 *  Mapper guards all access with its own mutex.
 *
 * \ingroup mola_mapper_grp
 */
class WorldModelState
{
public:
  WorldModelState();
  ~WorldModelState();

  // The graph state (GTSAM + ISAM2) is move-only; this whole container is a
  // single owned member of Mapper, so disable copying and moving.
  WorldModelState(const WorldModelState &) = delete;
  WorldModelState & operator=(const WorldModelState &) = delete;
  WorldModelState(WorldModelState &&) = delete;
  WorldModelState & operator=(WorldModelState &&) = delete;

  // PIMPL holding gtsam::Values, gtsam::NonlinearFactorGraph and gtsam::ISAM2.
  struct GtsamData;
  mrpt::pimpl<GtsamData> gtsam;

  /** @name Keyframes
   * @{ */

  /// Bidirectional timestamp <-> keyframe-id map.
  mrpt::containers::bimap<mrpt::Clock::time_point, KeyFrameID> time_to_kf_id;

  /// Raw observations / annotations for each keyframe.
  std::map<KeyFrameID, mrpt::obs::CSensoryFrame> keyframe_observations;

  /// Latest optimized state of one keyframe (pose in {map} + body twist +
  /// kinematic links to neighbor keyframes).
  struct FrameState
  {
    mrpt::poses::CPose3D pose;   //!< in the reference (map) frame
    mrpt::math::TTwist3D twist;  //!< in the local (body) frame
    std::set<KeyFrameID> kinematic_links_to;

    /// Cached marginal covariances (SE(3) [x y z yaw pitch roll] and twist
    /// [vx vy vz wx wy wz]). The optimizer fills these for the latest keyframe
    /// each solve so the query path (estimated_navstate / the high-rate
    /// publisher) never has to touch iSAM2. Empty when not yet computed.
    std::optional<mrpt::math::CMatrixDouble66> pose_cov;
    std::optional<mrpt::math::CMatrixDouble66> twist_cov;
  };

  /// Latest optimized per-keyframe states (updated after each iSAM2 solve).
  std::map<KeyFrameID, FrameState> last_estimated_states;

  /// Latest estimated frame transforms: T_map_to_odometry_frame_k by numeric id
  /// (and T_enu_to_map under REFERENCE_FRAME_ID=0 when geo-referencing).
  std::map<OdometryFrameID, mrpt::poses::CPose3DPDFGaussian> last_estimated_frames;

  /// The last raw pose received from each odometry source, expressed in that
  /// source's OWN frame {odom_i} (NOT in {map}), together with its timestamp.
  /// This is the anchor `estimated_navstate(t, {odom_i})` extrapolates from, so
  /// the short-term prediction stays continuous in the front end's own frame
  /// and immune to {map} corrections (geo-ref / loop closure / optimizer jitter)
  /// that would otherwise leak in if the pose were reconstructed globally as
  /// X(kf) (-) T_map_to_odom_i over the non-windowed central map.
  struct RawSourcePose
  {
    mrpt::Clock::time_point stamp;
    mrpt::poses::CPose3DPDFGaussian pose;  //!< in {odom_i}

    /// Body-frame twist (vx,vy,vz,wx,wy,wz) finite-differenced from THIS
    /// source's OWN consecutive raw poses in {odom_i}. The short-term predictor
    /// extrapolates the {odom_i} anchor with THIS twist, NOT the per-keyframe
    /// graph V(kf)/W(kf). Reason: the {odom_i} prediction must be FULLY
    /// frame-local. The graph twist is re-optimized every iSAM2 solve by the
    /// absolute factors (GNSS / IMU-gravity leveling / loop closure); once
    /// T_enu_to_map roll/pitch is pinned (so those factors bend the soft
    /// keyframe chain rather than tilt the transform), the latest keyframe's
    /// V/W swings per solve and that jitter would leak into the prediction --
    /// producing meter/degree jumps between consecutive scans that wreck a
    /// front end's ICP initial guess (observed on MulRan DCC01). The
    /// finite-difference twist is the source's OWN measured velocity, immune to
    /// those {map}-frame corrections, so the anchor + twist are both
    /// frame-local. Falls back to the graph V/W until two raw poses exist.
    mrpt::math::TTwist3D local_twist;
    bool has_local_twist = false;
  };
  std::map<OdometryFrameID, RawSourcePose> last_raw_pose_by_source;

  /// Topological connectivity for radius queries and loop-closure gating.
  KeyFrameConnectivity kf_connectivity;

  [[nodiscard]] bool empty() const { return time_to_kf_id.empty(); }

  [[nodiscard]] KeyFrameID last_kf_id() const;
  [[nodiscard]] KeyFrameID generate_new_kf_id() const;

  void add_kf_connectivity(KeyFrameID id1, KeyFrameID id2);

  /// Returns all keyframes within a topological distance of `id`.
  [[nodiscard]] std::set<KeyFrameID> get_keyframes_in_topological_radius(
    KeyFrameID id, size_t max_topological_distance = 0) const;

  /// Builds a CSimpleMap snapshot from the current keyframes and poses.
  [[nodiscard]] mrpt::maps::CSimpleMap as_simple_map() const;

  /** @} */

  /** @name Odometry frame registry
   * @{ */

  /// Bidirectional odometry frame-name <-> numeric-id map.
  mrpt::containers::bimap<std::string, OdometryFrameID> known_odom_frames;

  /// Monotonic counter for odometry frame IDs (>=1; 0 is reserved for ENU).
  OdometryFrameID next_odom_frame_id = 1;

  /** @} */

  /** @name Geo-referencing
   * @{ */

  /// Current geo-referencing (estimated or fixed), mirrored to/from the map.
  std::optional<mola::Georeferencing> geo_reference;

  /// First GNSS datum, used as the tentative ENU origin while estimating
  /// geo-referencing (before `geo_reference` is finalized).
  std::optional<mrpt::topography::TGeodeticCoords> tentative_geo_coord_reference;

  /** @} */

  /// Resets everything to an empty state.
  void clear();
};

}  // namespace mola::mapper
