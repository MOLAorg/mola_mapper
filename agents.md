# mola_mapper - AI agent context guide

Central 3D SLAM map for MOLA: fuses LIO/VIO/IMU/GNSS/wheels into ONE optimized
world model (keyframes as a `CSimpleMap` + a GTSAM factor graph), with anytime
loop closure, geo-referencing, lifelong keyframe management and relocalization.

Full plan, architecture rationale and task checklist: `~/plans/900_mola_mapper.md`.
Keep both this file and the plan in sync as the code changes. Do not mention
phase numbers in this repo's docs or code.

Use clang-format-14 on generated code.

## What it is

`mola::mapper::Mapper` implements `mola::NavStateFilter`,
`LocalizationSourceBase`, `MapSourceBase`, `DiagnosticsProvider`, and
`mola::SharedKeyframeMap`. It is the single source of truth for short-term
pose prediction that LIO/VIO query (replacing `mola_state_estimation_
{simple,smoother}` when used), and the sink front ends push sparse
central-map keyframes to via `requestInsertKeyframe()`. It is NOT a
`FrontEndBase`: it does no raw-scan ICP.

`mola_lidar_odometry` integrates both roles: it queries `navstate_fuse`
densely every scan, and separately pushes sparse keyframes through the
`SharedKeyframeMap` sink at its own keyframe-sparsity criterion, using a
dedicated source frame name (`publish_reference_frame + "_kf"`, distinct from
the dense path's frame) so the two paths don't collide.

## Layout

```
module/include/mola_mapper/   Public headers
  Mapper.h                     Main class
  Parameters.h                   YAML-loaded config (navstate group mirrors the smoother)
  WorldModelState.h              Central map state (keyframes, connectivity, geo-ref, GTSAM pimpl)
  ImuGravityFilter.h             Robust low-dynamics gravity-direction reducer
module/src/
  Mapper.cpp                   Lifecycle: initialize/spinOnce/reset/diagnostics + IMPLEMENTS_MRPT_OBJECT
  Mapper_Fusion.cpp            Keyframe management + factor-graph fusion + estimated_navstate
  Mapper_KeyframeIngestion.cpp SharedKeyframeMap sink: requestInsertKeyframe()
  Mapper_GUI.cpp               MolaViz/MolaVizImGui viz: KF tree, graph edges, per-source movable
                                  {odom_i} frames, {enu} geo-ref marker, status/geo-ref/view panel
  Mapper_SensorCallbacks.cpp   onNewObservation dispatch -> fuse_*()
  Mapper_LoopClosure.cpp         Background LC thread: snapshots the map, runs the
                                  mola_sm_loop_closure detector (analyze()) off-lock,
                                  merges accepted edges as robust BetweenFactors;
                                  plus the end-of-run finalize pass (batch full
                                  scans + re-optimization until no new loops)
  ImuGravityFilter.cpp           Robust low-dynamics gravity-direction reducer (pure/testable):
                                  given a window of lever-arm-corrected accel/gyro samples,
                                  rejects motion-contaminated ones, robustly averages the
                                  survivors -> ONE gravity dir + data-earned sigma
  WorldModelState.cpp            GtsamData pimpl (ISAM2/Values/NonlinearFactorGraph) + map helpers
  Parameters.cpp                 loadFrom(yaml)
  register.cpp                   MOLA_REGISTER_MODULE(mola::mapper::Mapper)
  GtsamData.h, factor_builders.h Private GTSAM symbol scheme + factor builders (shared by the two fusion TUs)
  covariance_utils.h             Shared SE(3)-covariance summary (max position / orientation sigma)
apps/mola-mapper-cli.cpp      Offline front end (skeleton)
params/mapper.yaml            Default config (no fixed geo-ref; pure-odometry-safe defaults)
mola-cli-launchs/                Live system YAMLs (KITTI, MulRan, BotanicGarden, Oxford Spires,
                                  ConSLAM, generic ROS 2 bag)
test/                            Unit tests (plain main() + MRPT ASSERT_ macros, run by mola_add_test)
docs/call-graph.md               Mermaid diagram tracing every public-API method to its internals
```

## Architecture invariants

- GTSAM is hidden behind `WorldModelState::GtsamData` (pimpl via
  `mrpt::make_impl`, kept copyable: `std::optional<gtsam::ISAM2>`, not
  `unique_ptr`).
- Solver: full `ISAM2` over ALL keyframes (the central map, not a fixed-lag
  window) + LM on loop closure + a lightweight predictor for short-term
  queries. Persistent variables (`T_enu_to_map`, `T_map_to_odom_i`) must stay
  out of any lossy fixed-lag marginalization.
- A background optimizer thread runs the iSAM2 solve off the query path
  (`enable_optimizer_thread`, default true in real launches, false for
  deterministic unit tests). `estimated_navstate()` /
  `get_latest_state_and_covariance()` read cached per-keyframe state, never
  touch iSAM2 directly. The solve is lock-split so heavy compute never blocks
  queries / publisher / ingestion. Always keep the thread on for any real
  high-rate source.
- The high-rate publisher (`spinOnce()` -> `publish_high_rate_pose()`)
  extrapolates the latest anchor with the kinematic model and publishes via
  `advertiseUpdatedLocalization()` at `high_rate_pose_publish_rate_hz`, gated
  on a subscriber existing.
- IMU never creates keyframes nor inserts per-sample factors: each raw
  reading is lever-arm-corrected to the vehicle frame and pushed into one
  global `LocalVelocityBuffer`; the gravity/attitude/gyro factors are built
  ONCE per real keyframe, draining that buffer's window since the previous
  keyframe.
- High-rate wheels are aggregated the same way when
  `aggregate_high_rate_into_edges` is on: no `{odom_wheels}` frame variable,
  one relative-pose edge per keyframe transition from the net wheel motion.
- Odometry is fused as a SINGLE chain of CONSECUTIVE relative-pose edges, not
  absolute `Between(F(i), T(kf))` ties — this is the key fix for `{map}`
  tilt/z deformation. Every time-adjacent keyframe pair, regardless of
  source, is linked by exactly one `BetweenFactor`. `T_map_to_odom_i` is a
  DERIVED, instantaneous readout each solve, not a fusion unknown.
- Chain-edge noise is DATA-DRIVEN from the propagated relative covariance
  (faithful port of `mola_sm_loop_closure::add_odometry_edges`), not a
  hardcoded isotropic sigma — this is what lets the absolute IMU-gravity/GNSS
  factors actually level the map.
- `estimated_navstate(t, {odom_i})` is frame-local: anchored on the source's
  own last raw pose, extrapolated with a finite-difference twist from that
  source's own consecutive raw poses (kinematic-model-aware). It does NOT
  reconstruct through `{map}` and does NOT use the graph `V(kf)/W(kf)` (both
  are re-jittered every solve by absolute factors, which would otherwise leak
  into the short-term prediction LIO/VIO depend on for ICP initial guesses).
  Falls back to the global conversion only before a source's first
  `fuse_pose()` lands.
- The extrapolation VELOCITY is low-passed (`predict_twist_filter_enabled`,
  `predict_twist_filter_time_const`, dt-aware EMA in `TwistLowPass`), on both
  velocity sources: the newest keyframe's graph `V/W` and each source's
  `local_twist`. Raw, either one hands a jittery motion prior to LIO, which
  starts ICP from a bad guess and drops scans under real-time load. A
  non-advancing timestamp must leave the EMA untouched (the solver re-runs at
  an unchanged newest-keyframe stamp; refreshing there would wipe the history
  and pass the raw value through). Paired with moderate
  `sigma_random_walk_acceleration_*` defaults (0.5 / 1.0): a loose angular
  sigma lets the boundary keyframe's yaw rate swing and rotate the prediction.
- `estimated_navstate()` never throws: it degrades to `nullopt`. A keyframe
  exists in `time_to_kf_id` at CREATION but only in `last_estimated_states`
  after a solve commits, so the query path routinely sees not-yet-solved
  keyframes (`get_latest_state_and_covariance()` would throw). The body lives
  in `estimated_navstate_impl()` so the public entry point can catch.
- Out-of-order keyframe guard (mandatory): in
  `create_or_get_keyframe_by_timestamp_locked()`, a request older than the
  newest keyframe snaps to the nearest existing keyframe instead of inserting
  a past variable.
- Frame model mirrors `mola_state_estimation_smoother`: one `{odom_i}` per
  source, plus `{map}` and `{enu}`; the graph estimates `T_enu_to_map`.
- Keyframe-creation gating (`KeyframeCreationSource`: Auto / SharedMapOnly /
  SensorClock): only `requestInsertKeyframe()` (or a sensor-only clock with
  no LIO/VIO producer) creates graph keyframe variables. Dense `fuse_pose()`,
  rate-capped IMU/wheels, and GNSS feed the predictor anchor and/or
  accumulate into inter-keyframe constraints; they never spawn a variable.
  `Auto` mode runs legacy per-call creation until the first
  `requestInsertKeyframe()` lands, then flips to `SharedMapOnly`.
- Geo-ref convergence is MODE-AWARE, and the two modes use different criteria.
  Live (`estimate_geo_reference: true`): `optimize_and_refresh()` publishes
  `state_.geo_reference` only once `T_enu_to_map`'s own position+orientation
  sigmas (plus the latest keyframe's ORIENTATION sigma) clear
  `convergence_max_position_sigma` / `convergence_max_orientation_sigma_deg`,
  and `has_converged_localization()` is then sticky on that. It deliberately
  does NOT gate on the keyframe's ABSOLUTE position sigma: the central map
  pins a far-away gauge anchor, so that grows with distance and would block
  convergence forever on long trajectories. Relocalize
  (`estimate_geo_reference: false` + a fixed geo-ref): the geo-reference is
  known up front and says nothing about being localized, so
  `has_converged_localization()` gates on the latest keyframe's own pose
  sigma. Gating on `estimate_geo_reference` alone made relocalize mode
  unconditionally "never converged".
- `set_geo_reference()` (front end loaded a geo-referenced map) MUST be
  implemented, not inherited: an unpinned `T_enu_to_map` leaves the absolute
  rotation a gauge freedom (gravity/attitude factors only measure rotation
  relative to it) and iSAM2 throws. With no keyframes yet it rebuilds the
  graph (after `state_.clear()`, or the still-pending `symbol_T_enu_to_map`
  is inserted twice); with keyframes present it pins the existing variable
  with an extra prior, since the central map must survive the call.
- `reset()` resets only the short-term per-source integration anchors
  (wheel/IMU/ingestion chains), never the keyframes/graph/geo-ref/diagnostic
  counters — the central map is shared, persistent state and must survive one
  front end relocalizing.

- Loop closure is a LIBRARY, not a running module: the `mola_sm_loop_closure`
  F2F engine is linked and driven from a mapper-owned background thread
  (`loop_closure_enabled`, off by default). The thread snapshots the central
  map, runs the detector-only `analyze()` OFF the state lock (streaming +
  abortable), and merges accepted edges as robust (Huber) `BetweenFactor`s,
  then wakes the optimizer. Keyframes carry the raw lidar scan plus a "metadata"
  comment observation with the per-keyframe velocity window, so the LC pipeline
  can deskew when the sensor provides per-point timestamps (use a deskew-free
  pipeline for sensors that don't). GNC-in-parallel and the LC-event
  notification to front ends are still open.

- LC pipeline config lives in `params/loop-closure-f2f-mapper.yaml`, which
  `$import`s the package's f2f pipeline and overrides ONLY the ICP registration
  core (point-to-point instead of cov2cov; wider initial pairing sigma), so
  mola_sm_loop_closure keeps its own defaults. Both are needed for real loops to
  pass acceptance at all. The KITTI, MulRan and Oxford Spires launchers enable LC
  by default and point at it.

- Dense high-resolution LiDARs need a wider ICP FINAL pairing sigma too. The base
  pipeline evaluates ICP quality as the paired-point ratio at the final annealed
  threshold (`force_final_pairings_for_quality`, default 0.05 m). On the dense
  Hesai clouds of Oxford Spires, even a correct cross-pass alignment lands few
  points within 5 cm, so quality collapses to ~0% and every loop is rejected. The
  Oxford Spires launcher therefore points at `params/loop-closure-f2f-mapper-
  oxford-spires.yaml`, which `$import`s the shared mapper override and only raises
  `threshold_sigma_final` to 0.3 m (env `LC_ICP_FINAL_SIGMA`). This is a
  registration-resolution setting for a dense sensor, NOT a change to the
  `min_icp_goodness` acceptance level; KITTI/MulRan keep 0.05 m. Validated on
  observatory-quarter-01 (real-time, zero drops): 5 loops closed, APE RMSE 0.47 m
  vs 1.53 m with LC off. Note the real-time pipeline is non-deterministic (async
  optimizer + LC threads, scan drops under time-warp), so the absolute APE varies
  run-to-run (~0.25-0.5 m LC-on in good runs); LC helps in every fair
  same-playback comparison. Its default also restricts LC candidates to nearby
  revisits (`max_distance_for_lc_candidate` 15 m) for a real-time accuracy/compute
  balance; far pairs mostly add cost since single-scan ICP only bridges its basin.

- LC can also serve as an offline map-refinement stage: a dense keyframe graph
  plus aggressive nearby-candidate generation yields thousands of edges and the
  best accuracy, but is much heavier (OFF by default, enabled via the env-var
  bundle documented in the Oxford Spires launcher header). Denser keyframes or
  denser candidates ALONE regress; both together win.

- Online LC scans alone close few loops: a pair only closes once BOTH endpoints
  exist AND drift is small enough, which for big revisits happens near the end of
  the run. Hence `loop_closure_finalize_rounds` (default 8): at destruction,
  repeat full scan + synchronous re-optimization until a round finds nothing new,
  each round's correction bringing further candidates into ICP's basin. Rounds
  pass the already-closed pairs via `LoopClosureAnalyzeOptions::exclude_pairs`,
  without which a scan just re-proposes them and burns its candidate budget.
  Validated on KITTI-00: 8 loops closed (4 online + 4 in finalize, then
  converged), vs 0 loops and 8.95 m absolute pose error with LC off.

See `~/plans/900_mola_mapper.md` for the rationale behind each of these (the
real-data failure modes that drove each design choice), the GNC-bootstrap
geo-ref rewire, IMU preintegration, remaining loop-closure work (GNC, LC
event), save/load, relocalization and spatial-paging designs that are not yet
implemented, and the full task checklist.

## Build / test

```bash
cd ~/ros2_ws
colcon build --packages-select mola_mapper
colcon test  --packages-select mola_mapper && colcon test-result --verbose
```

## Retuning an imported pipeline: prefer `$define`

When a launcher `$import`s `mola_lidar_odometry`'s pipeline YAML into the
`lidar_odom` module, prefer `$define` (mola_yaml) over a sibling override to
retune it: it binds the `${VAR|default}` hooks the imported file already
exposes, with priority `environment > $define > the file's inline default`.
The Oxford Spires and ConSLAM launchers use it for `MOLA_DESKEW_METHOD` and
`MOLA_LO_INITIAL_LOCALIZATION_METHOD`.

This avoids two traps that both previously landed a launcher silently running
`FixedPose` and linear deskew. First, a sibling override must match the
imported file's OWN nesting: `initial_localization` is a TOP-LEVEL key there
(sibling of `params:`, not inside it), so nesting the override under the
launcher's `params:` lands it in an unused `params.initial_localization`
(deep-merge only reaches keys at the same nesting level). Second,
`observations_deskew_pass` is a YAML sequence, which deep-merge replaces
wholesale rather than patching, so overriding one field used to require
duplicating the whole filter step verbatim. Use `$define` when the setting has
a hook; fall back to a sibling override (at the right nesting level) when it
does not, and verify the result with `mola-yaml-parser` on the merged config.

## Real-dataset runs

```bash
export KITTI_BASE_DIR=/path/to/kitti_root
export MOLA_ODOMETRY_PIPELINE_YAML=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml
KITTI_SEQ=04 MOLA_LINK_FIRST_POSE_SIGMA=1e-6 \
  mola-cli mola-cli-launchs/lidar_odometry_mapper_from_kitti.yaml
# MOLA_WITH_GUI=false for headless; MOLA_TIME_WARP=N to speed up/slow down.
```

```bash
export OXFORD_SPIRES_ROSBAG2=/path/to/sequence/raw/ros2bag/<segment>
MOLA_WITH_GUI=false MOLA_MAPPER_TUM_TRAJECTORY_OUTPUT=/tmp/obs01.tum \
  mola-cli mola-cli-launchs/lidar_odometry_mapper_from_oxford_spires.yaml
# LiDAR+IMU only (no GNSS). Loop closure is ON by default (Hesai has per-point
# timestamps, so the LC pipeline deskews normally). GT trajectories ship as TUM
# under each sequence's processed/trajectory/gt-tum.txt.
# Multi-segment sequences: override rosbag_filename in the YAML with a list
# (Rosbag2Dataset supports multi-bag playback).
```

```bash
export BOTANICGARDEN_LIO_BAG=$HOME/datasets/botanic/1018_00_LIO.bag
export MOLA_ODOMETRY_PIPELINE_YAML=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml
mola-cli mola-cli-launchs/lidar_odometry_mapper_from_botanicgarden.yaml
```
Reads a ROS 1 `.bag` directly via `mola::Rosbag1Dataset` (`mola_input_rosbag1`,
no ROS 1 install needed); use that package's `rosbag1-info <bag>` CLI to
confirm a dataset's sensor inventory before wiring a new launcher (don't trust
a dataset's own README blindly).

```bash
export CONSLAM_BAG=$HOME/datasets/ConSLAM/sequence2.bag
mola-cli mola-cli-launchs/lidar_odometry_mapper_from_conslam.yaml
```
Hand-held-scanner construction dataset (Velodyne VLP-16 + Xsens MTi-610 IMU,
no GNSS): another pure LiDAR+IMU `Rosbag1Dataset` case. The dataset's own
extrinsic calibration files (`calib_lidar2imu.txt` etc.) ship in a separate
`data_calib.zip`, not in the software repo or a plain sequence `.bag`
download, so the LiDAR/IMU `fixed_sensor_pose` only encodes the rotation the
paper documents (180 deg yaw between the two mounting frames); translation
defaults to zero pending the real calibration file.

`lidar_odometry_mapper_from_rosbag.yaml` is the generic entry point for any
ROS 1 or ROS 2 bag: lidar/imu/gps/wheel-odometry topics are all optional, set via
`MOLA_LIDAR_TOPIC`/`MOLA_IMU_TOPIC`/`MOLA_GNSS_TOPIC`/`MOLA_ODOMETRY_TOPIC`
(unset = that sensor is skipped, no crash).

MulRan launcher (`lidar_odometry_mapper_from_mulran.yaml`,
`MULRAN_BASE_DIR=... MULRAN_SEQ=DCC01`) is the LiDAR+IMU+GNSS reference case;
self-contains `estimate_geo_reference: true` and
`link_first_pose_to_reference_origin_sigma: 1e-6`.

A pure-odometry or `estimate_geo_reference` run needs
`link_first_pose_to_reference_origin_sigma` set (e.g. `1e-6`), or the whole
graph stays gauge-free and `LidarOdometry` discards its motion model.

## Call graph

`docs/call-graph.md` traces every major public-API method down to its
internal helpers, state stores, and the background optimizer. Keep it in sync
whenever you add/rename/rewire a public method, the call chain inside
`fuse_pose_locked()` / `request_insert_keyframe_locked()` /
`optimize_and_refresh()` / `estimated_navstate()`, a new hot-path helper, the
`sensor_kf_creation_allowed()` gate, or the locking model.

## Code style

clang-format-14; no one-line `if`; one variable per line; no em/en dashes;
American spelling; anonymous namespaces over `static`. clang-tidy per the
repo `.clang-tidy`. Don't sign commits as an AI agent. Keep this file, the
plan, and `docs/call-graph.md` in sync with the code.
</content>
