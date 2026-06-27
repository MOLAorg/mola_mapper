# mola_mapper_3d - AI agent context guide

Central 3D SLAM map for MOLA: fuses LIO/VIO/IMU/GNSS/wheels into ONE optimized
world model (keyframes as a `CSimpleMap` + a GTSAM factor graph), with anytime
loop closure, geo-referencing, lifelong keyframe management and relocalization.

Full plan + task checklist (keep it in sync as you work):
`~/plans/900_mola_mapper_3d_plan.md`. Do NOT mention phase numbers in this repo
docs or code.

## What it is

`mola::mapper_3d::Mapper3D` implements `mola::NavStateFilter`,
`LocalizationSourceBase`, `MapSourceBase`, `DiagnosticsProvider`, and
`mola::SharedKeyframeMap` (new `mola_kernel` interface, see below). It is the
single source of truth for short-term pose prediction that LIO/VIO query
(replacing `mola_state_estimation_{simple,smoother}` when used), AND the sink
front ends push sparse central-map keyframes to via `requestInsertKeyframe()`.
It is NOT a `FrontEndBase`: it does no raw-scan ICP.

`mola_lidar_odometry` already integrates both roles (see its own agents.md):
it queries `navstate_fuse` densely every scan (unchanged, pre-existing), and
separately pushes sparse keyframes through the `SharedKeyframeMap` sink at
its own keyframe-sparsity criterion (`LidarOdometry_ProcessScan.cpp::
pushKeyframeToSharedKeyframeMap()`), using a DEDICATED source frame name
(`publish_reference_frame + "_kf"`) so the two paths don't collide (see "Real
end-to-end validation lessons" below).

## Layout

```
module/include/mola_mapper_3d/   Public headers
  Mapper3D.h                     Main class
  Parameters.h                   YAML-loaded config (navstate group mirrors the smoother)
  WorldModelState.h              Central map state (keyframes, connectivity, geo-ref, GTSAM pimpl)
module/src/
  Mapper3D.cpp                   Lifecycle: initialize/spinOnce/reset/diagnostics + IMPLEMENTS_MRPT_OBJECT
  Mapper3D_Fusion.cpp            Keyframe management + factor-graph fusion + estimated_navstate
  Mapper3D_KeyframeIngestion.cpp SharedKeyframeMap sink: requestInsertKeyframe() (anchor-once + consecutive chain)
  Mapper3D_GUI.cpp               Optional MolaViz/MolaVizImGui viz: KF tree + graph edges + per-source movable {odom_i} frames + the {enu} geo-ref frame marker (drawn at inverse(T_enu_to_map) while geo-referencing) + GuiWidgetDescription panel (Status / Geo-ref / View tabs: KF/edge/IMU-factor counts, geo-ref T_enu_to_map + GNSS factors + per-source T_map_to_odom drift as trans/rot, now the INSTANTANEOUS derived transform -- see the T_map_to_odom_i design note). The View tab has a "Viz reference frame" combo (map / enu) selecting the scene origin: it applies a `vizXform` (identity for map, `T_enu_to_map` for enu) to all scene containers + movable odom frames, so "enu" renders the map North-oriented (default; {enu} always exists as the identity weak-prior until geo-ref converges). An XYZ "enu" corner is drawn at the scene origin only while the enu frame is selected. NOTE: odom drift is ~0 on pure-odometry runs (no GNSS => {map} == LIO's {odom}); once GNSS/IMU geo-referencing pulls/levels the map the ROTATION grows to the real map-vs-odom angle (e.g. ~11 deg on DCC01) and the TRANSLATION is its lever-arm consequence over the trajectory (100s of m), NOT a sign the map is wrong (DCC01 map is ~6.7 m RMSE to GT).
  Mapper3D_SensorCallbacks.cpp   onNewObservation dispatch -> fuse_*()
  WorldModelState.cpp            GtsamData pimpl (ISAM2/Values/NonlinearFactorGraph) + map helpers
  Parameters.cpp                 loadFrom(yaml)
  register.cpp                   MOLA_REGISTER_MODULE(mola::mapper_3d::Mapper3D)
  GtsamData.h, factor_builders.h Private GTSAM symbol scheme + factor builders (shared by the two fusion TUs)
apps/mola-mapper-3d-cli.cpp      Offline front end (skeleton)
params/mapper-3d.yaml            Default config (no fixed geo-ref; pure-odometry-safe defaults)
mola-cli-launchs/                Live system YAMLs:
  lidar_odometry_mapper3d_from_kitti.yaml    LIO+Mapper3D on KITTI, GUI on by default (mola::MolaVizImGui,
                                              `enabled: ${MOLA_WITH_GUI|true}`); validated end-to-end (see below)
  lidar_odometry_mapper3d_from_mulran.yaml   + IMU/GNSS (MulRan); headless; prepared, NOT yet run on real data
  lidar_odometry_mapper3d_from_botanicgarden.yaml  LiDAR+IMU+wheels (ROS1 .bag via mola_input_rosbag1,
                                              no GNSS in this dataset); GUI on by default; validated end-to-end
test/                            Unit tests (plain main() + MRPT ASSERT_ macros, run by mola_add_test)
  test-keyframe-ingestion.cpp    SharedKeyframeMap sink: basic plumbing + IMU-corrects-drift scenario
  test-georef-convergence.cpp    60-case synthetic odom+GNSS sweep: T_enu_to_map convergence,
                                  has_converged_localization(), with/without GNSS
```

## Key design notes (and why)

- GTSAM is hidden behind `WorldModelState::GtsamData` (pimpl via `mrpt::make_impl`)
  to keep it out of the public header. The pimpl must stay **copyable**, so use
  `std::optional<gtsam::ISAM2>`, not `unique_ptr`.
- Solver: full `ISAM2` over ALL keyframes (the central map, not a fixed-lag
  window) + LM on loop closure + a lightweight predictor for short-term queries.
  Persistent variables (`T_enu_to_map`, `T_map_to_odom_i`) must stay OUT of any
  lossy fixed-lag marginalization.
- **Background optimizer thread** (`enable_optimizer_thread`, default true in the
  real launches / shared `params/mapper-3d.yaml`; the C++ default is false so
  unit tests stay deterministic single-threaded). The growing iSAM2 solve runs
  on its own thread, OFF the query path. The hot query path
  (`estimated_navstate()` / `get_latest_state_and_covariance()`) is iSAM2-free:
  it reads cached per-keyframe poses/twists + the latest keyframe's cached
  marginal covariance (`FrameState::pose_cov`/`twist_cov`, filled by the
  optimizer each solve). The solve is **lock-split** (`optimize_and_refresh()`,
  three phases: grab-pending under `stateMutex_` -> heavy iSAM2 work holding only
  `solve_mutex_`, NOT `stateMutex_` -> commit caches under `stateMutex_`), so the
  heavy compute never blocks queries / the publisher / ingestion. Fusion wakes
  the thread via a condition variable (`notify_optimizer()`); a 50 ms safety-net
  timeout covers a missed wakeup. Destructor/`initialize()` stop+join with NO
  lock held (the thread itself takes `stateMutex_`, so joining under it would
  deadlock). **Measured on BotanicGarden** (real `mola-cli`, 1440 scans):
  per-scan `estimated_navstate()` 174 ms avg -> **35 us avg / 259 us max**.
  When `enable_optimizer_thread:false`, `estimated_navstate()` flushes+solves
  synchronously so every query reflects all data fused so far (tests rely on
  this).
- **High-rate publisher**: `spinOnce()` -> `publish_high_rate_pose()` publishes
  the latest extrapolated reference-frame pose via
  `advertiseUpdatedLocalization()` at `high_rate_pose_publish_rate_hz` (gated on
  a subscriber existing). This is the first thing that actually drives the
  `LocalizationSourceBase` output.
- **High-rate IMU/wheels aggregation** (`aggregate_high_rate_into_edges`, opt-in;
  default false so unit tests are unchanged; true in the BotanicGarden launch):
  IMU and wheels no longer each spawn a keyframe per sample. They SHARE a
  bounded-rate keyframe clock (`sensor_keyframe_min_period`, default 0.5 s), and
  wheel odometry is aggregated into ONE relative-pose edge
  `Between(T(prev_kf), T(cur_kf))` per keyframe transition (the "consecutive
  frame edge" model; no `{odom_wheels}` frame variable created). Synthetic test:
  201 IMU+wheel samples -> 9 keyframes with the trajectory recovered
  (`test-highrate-aggregation`). On BotanicGarden the IMU/wheels now reuse LIO's
  denser scan keyframes, adding ~zero extra. See `Mapper3D_Fusion.cpp::
  fuse_odometry` / `fuse_imu`.
- **Graph growth is still only PARTIALLY bounded** (known): the IMU/wheels
  explosion is fixed (above), but LIO's DENSE per-scan `fuse_pose()` (used for
  short-term prediction, plan 6.2b) still creates a keyframe per scan
  (~10 Hz, ~990 KFs for a 150 s BotanicGarden run) and is NOT aggregated. The
  optimizer thread keeps queries fast regardless (37 us avg there). Remaining
  fixes: full gtsam IMU preintegration + bias (plan 4.12), predictor/central-map
  separation for the dense LIO path (6.2b), and spatial paging (4.11 / Phase 10).
- Out-of-order keyframe guard (mandatory): in
  `create_or_get_keyframe_by_timestamp_locked()`, a request older than the
  newest keyframe snaps to the nearest existing keyframe instead of inserting a
  past variable (iSAM2 marginalization needs non-decreasing keyframe stamps).
- Frame model mirrors `mola_state_estimation_smoother`: one `{odom_i}` per
  source, plus `{map}` and `{enu}`; the graph estimates `T_enu_to_map` (a real
  GNSS/IMU-estimated unknown). Reuse `mola_gtsam_factors` for kinematic/IMU/GNSS
  factors.
- **Odometry is fused as a SINGLE chain of CONSECUTIVE relative-pose edges, NOT
  as absolute `Between(F(i), T(kf))` ties** (`Mapper3D::link_into_odometry_chain_locked`,
  shared by the dense `fuse_pose()` path AND the `SharedKeyframeMap` sink). This
  is the crux fix for catastrophic `{map}` tilt/z deformation (see below). Each
  keyframe stores the absolute odometry pose that defined it (`kf_odom_abs_pose_`,
  first-writer-wins); every TIME-ADJACENT keyframe pair (regardless of source) is
  linked by exactly ONE `BetweenFactor(T(prev), T(next)) = odom(next) (-) odom(prev)`
  -- the `mola_sm_loop_closure::add_odometry_edges` pattern, never skipping a
  keyframe so no conflicting loop edges form. New keyframes are dead-reckon
  initialized (`T(prev_estimate) (+) relative_odom`) so the long relative chain,
  pinned only at a far gauge anchor, stays in the correct (un-twisted) iSAM2
  basin. **Why not absolute ties:** the smoother ties every reading to one rigid
  `F(i)=T_map_to_odom_i` in a SHORT fixed-lag window where that is valid; over
  the WHOLE central map a single rigid `F(i)` cannot fit a drifting source's
  trajectory, so hundreds of absolute ties become mutually inconsistent and the
  optimizer mangles `{map}` (observed on MulRan DCC01: ~80 deg tilt, +-300 m z,
  vs ~6.7 m-RMSE-to-GT / <=12 deg after the fix; `evo_ape -a` vs `global_pose.csv`).
  Relative edges are also frame-invariant, so multiple odometry sources fuse by
  consensus on the shared keyframe chain without needing `F(i)` to absorb a rigid
  inter-frame offset.
- **`T_map_to_odom_i` is a DERIVED, INSTANTANEOUS readout, not a fusion unknown.**
  It is recomputed each solve as `T(latest_kf_i) (+) inverse(odom_pose_i(latest_kf_i))`
  (`optimize_and_refresh()`, using `latest_kf_by_odom_frame_`), so it correctly
  drifts over time for a drifting source. The graph variable `F(i)` (i>=1) is now
  only a determinate, weak-prior'd placeholder pinned by a SINGLE one-time
  first-keyframe gauge anchor (needed so the first keyframe is determinate when no
  `link_first_pose_to_reference_origin` is set); it is never read. Consumed by the
  GUI drift readout and the per-source movable viz frame. NOTE: the readout's
  TRANSLATION is lever-arm-dominated (an N-degree map-vs-odom rotation over a
  multi-km trajectory shows as 100s of m of transform translation even though the
  map matches GT to a few m); the ROTATION component is the meaningful indicator.
- **`estimated_navstate(t, {odom_i})` is frame-local, NOT reconstructed through
  `{map}`** (`Mapper3D_Fusion.cpp`). A non-reference-frame prediction anchors on
  the source's OWN last raw pose in `{odom_i}` (stored per source in
  `WorldModelState::last_raw_pose_by_source` by `fuse_pose_locked()`) and
  extrapolates it forward by the body-twist increment via `body_twist_delta()`
  (which honors `kinematic_model`: full SE(3) `exp` for ConstantVelocity, the
  planar arc mirroring `FactorTricycleKinematic` for Tricycle). It does NOT
  compute `X(kf) (-) T_map_to_odom_i`. Why: over the NON-windowed central map a
  single rigid `T_map_to_odom_i` cannot represent the non-rigid {map}
  deformation that geo-ref/IMU/loop-closure apply, and the background optimizer
  re-jitters both `X(kf)` and `T_map_to_odom_i` every solve -- so the global
  reconstruction leaks meter/degree jumps into the short-term prediction. On
  MulRan DCC01 that wrecked LIO's ICP initial guess right after geo-ref
  convergence (ICP goodness collapsed 89%->0% permanently, 1215 zero-goodness
  scans); the frame-local anchor fixed it (0 zero-goodness scans, ICP holds
  ~85%). The smoother uses the same `X(kf) (-) T_map_to_odom` code but its short
  fixed-lag window keeps `{map}` ~ `{odom_i}` locally, so the leak stays
  negligible there; this central-map (non-windowed) variant is what makes it
  catastrophic. The frame-local prediction covariance is also built locally
  (raw-anchor cov + constant-accel growth over dt), NOT from the `{map}`-frame
  keyframe covariance, which grows unboundedly with gauge distance and would
  make front ends reject the motion model. Falls back to the global conversion
  only before this source's first `fuse_pose()` lands (when `{map}=={odom_i}`).
  The frame-local path is NOT gated on keyframe proximity: it only needs the
  source's own fresh raw anchor + a twist, so it still serves a prediction when
  the sparse central-map keyframes leave a gap > `max_time_to_use_velocity_model`
  (only the reference-`{map}` path and the no-raw-anchor fallback re-apply that
  proximity check). Gating the whole query on KF proximity previously returned
  `nullopt` on those gaps, starved LIO's motion model ("Not able to use velocity
  motion model"), and eventually froze it on MulRan DCC01 with sparse KFs.
- **TEMPORARY WORKAROUND (plan 4.7 investigation): the predictor extrapolates
  with a FINITE-DIFFERENCE twist from each source's consecutive raw poses
  (`RawSourcePose::local_twist`), not the graph `V(kf)`/`W(kf)`.** With IMU on,
  the graph body-twist variables came out wrong on MulRan (V ~0.2 m/s vs ~10
  true, W ~42 rad/s), so the prediction was near-static and LIO appeared frozen.
  Architecturally V/W are body-local + chain-tied and SHOULD be correct/IMU-
  immune; the corruption is unexplained -- see the plan 4.7 note. Trace with
  `MOLA_MAPPER3D_TRACE_PREDICT=1`. Remove the `has_local_twist` branches once the
  graph V/W is fixed.
- Closest existing templates to study: `mola_mapper_2d` (structure + 2D
  pose-graph SLAM) and `mola_state_estimation_smoother` (multi-frame fusion,
  factor builders, FastPredictor).
- `current_georeferencing()` / `has_converged_localization()` are
  CONVERGENCE-GATED when `estimate_geo_reference: true` (no
  `fixed_geo_reference`): `state_.geo_reference` is only published once BOTH
  the `T_enu_to_map` estimate AND the latest keyframe's vehicle-pose sigma in
  `{map}` clear `convergence_max_position_sigma` /
  `convergence_max_orientation_sigma_deg` (`process_pending_gtsam_updates_locked()`
  in `Mapper3D_Fusion.cpp`, mirrors the smoother). Yaw converges markedly
  slower than position when GNSS only gives position (heading comes
  indirectly from trajectory shape): don't be surprised if
  `convergence_max_orientation_sigma_deg` needs to be looser than position's
  threshold suggests for a given motion profile (see
  `test-georef-convergence.cpp`'s tuning note).
- `SharedKeyframeMap::requestInsertKeyframe()` feeds the SAME unified odometry
  chain as the dense `fuse_pose()` path: it forwards the request's keyframe +
  `pose_in_source.mean` into `link_into_odometry_chain_locked()` (consecutive
  relative edges, see the odometry-fusion note above). It does NOT keep its own
  per-source chain (the previous per-source chaining let the dense and sparse
  paths skip each other's keyframes and emit CONFLICTING relative edges between
  shared keyframe variables -- e.g. `T675->T677 = 5.30 m` vs
  `T675->T676->T677 = 2.12 m` -- which deformed `{map}`).
- **Chain edge noise is DATA-DRIVEN from the propagated relative covariance**
  (`add_odom_chain_edge_locked`, faithful port of
  `mola_sm_loop_closure::add_odometry_edges`). The per-DOF sigma is
  `sqrt(diag(cov_to (-) cov_from)) * odometry_edge_uncertainty_multiplier`, plus
  an additive floor `odometry_edge_min_sigma_xyz/ang_deg` (defaults match
  sm_loop_closure: mult 1.0, floor 1e-3 m / 1e-3 deg). Each keyframe therefore
  stores the FULL source pose PDF (`kf_odom_abs_pose_` is now a
  `CPose3DPDFGaussian`, fed from `pose_in_source` / the sanitized `fuse_pose`
  PDF), not just the mean. This leaves the drift-prone DOFs (z, roll, pitch) as
  soft as the source's own covariance says, so the absolute IMU-gravity / GNSS
  factors can level the map -- the hardcoded isotropic
  `keyframe_ingestion_sigma_lin/ang_deg` (used before, and still used for the
  one-time first-keyframe `F(i)` gauge tie) pinned them rigidly and blocked
  leveling. The additive floor bounds pathological near-zero input covariances
  (e.g. a relocalization seed). On DCC01 LIO+IMU this took map z-drift from
  ~37.6 m to ~3.6 m (GT z-span ~2 m). The factor MEAN still uses the exact
  relative pose; only the noise is data-driven.

## Real end-to-end validation lessons (KITTI, via real `mola-cli`, not unit tests)

Running `mola-cli-launchs/lidar_odometry_mapper3d_from_kitti.yaml` for real
(LIO + Mapper3D wired through the actual `findService<>()` module-loading
path) surfaced bugs the synthetic unit tests never hit. If you touch
`fuse_pose_locked()`, `add_kinematic_factor_between()`, or
`request_insert_keyframe_locked()`, re-read this before "fixing" something:

1. **Never reuse a dense-fusion `frame_id` as a `SharedKeyframeMap`
   `source_frame_id`.** A front end calling both `fuse_pose()` (every scan)
   and `requestInsertKeyframe()` on the SAME frame name lets the sink's
   anchor-once tie collide with the dense path's tie on the same
   (relocalization-seeded) keyframe -> `gtsam::IndeterminantLinearSystemException`
   at startup. Use a distinct name (LIO uses `publish_reference_frame + "_kf"`).
2. **Floor any `pose_in_source`/`fuse_pose()` input covariance; never use it
   raw.** Real front ends report pathological values (LIO's `FixedPose`
   relocalization seed pins `cov` at `1e-12`); mixed with weak priors elsewhere
   in the graph, that ill-conditions iSAM2's Cholesky. The consecutive-edge
   chain now DERIVES its per-DOF noise from the propagated input covariance (so
   z/tilt can be soft for leveling -- see the odometry-fusion note), but always
   adds the `odometry_edge_min_sigma_*` floor on top, which bounds those
   pathological near-zero inputs. Other direct priors (e.g. the `{map}`-frame
   relocalization seed, absolute pose priors) still use configured/floored sigma
   (`MIN_POSE_SIGMA_LIN/ANG` in `Mapper3D_Fusion.cpp`).
3. **Always difference `mrpt::Clock::time_point`s with
   `mrpt::system::timeDifference()`, never `toDouble(a) - toDouble(b)`.** The
   naive subtraction silently underflows (wraps to ~+1.8e12) for timestamps
   before the Unix epoch, which happens whenever a dataset's first timestamp
   is near zero (KITTI) and something (e.g. relocalization seeding) creates a
   keyframe a couple of ms earlier.
4. **A pure-odometry or estimate_geo_reference run needs
   `link_first_pose_to_reference_origin_sigma` set** (e.g. `1e-6`), or the
   whole graph is gauge-free, `estimated_navstate()` returns ~1e6-sigma
   absolute poses, and `LidarOdometry` permanently discards its motion model
   ("Discarding motion model ... due to large uncertainty"), starving both
   the local map and the `SharedKeyframeMap` sink. This is config, not code.
5. The keyframe-sparsity criterion (`distance_enough_sm` in
   `LidarOdometry_ProcessScan.cpp`) must drive its distance-checker `insert()`
   independently of `params_.simplemap.generate`, or a `SharedKeyframeMap`
   sink with self-simplemap-writing disabled silently gets only ONE keyframe
   ever: the checker reports "far enough" on its first-ever call (empty), but
   if nothing then inserts into it (because that was gated on
   `simplemap.generate`), it stays empty and keeps reporting zero distance
   forever after, so the sparsity criterion never fires again.

GUI runs confirmed working too: `mola-cli-launchs/lidar_odometry_mapper3d_from_kitti.yaml`
now ships with a `viz: mola::MolaVizImGui` module (`enabled: ${MOLA_WITH_GUI|true}`,
the standard `mola_launcher` per-module `enabled` flag) on by default.

6. **Bound any high-rate IMU/wheel-odometry feeding mapper3d directly via the
   max-insert-rate caps (`imu_max_insert_rate_hz` / `odometry_max_insert_rate_hz`,
   both default `5.0` Hz; `0` = insert every reading).** These REPLACED the older
   `imu_min_sample_period` / `odometry_min_sample_period` decimation knobs
   (2026-06-26): a positive max RATE is the natural spelling and lets us
   SUMMARIZE rather than drop. The IMU path buffers samples and inserts at most
   N summarized observations/second -- each carries the AVERAGED accelerometer
   (less-noisy gravity/leveling), averaged angular velocity, and the latest
   absolute orientation -- bounding BOTH the inserted-factor rate AND the
   IMU-driven keyframe-creation rate (keyframe-reuse window = 1/rate). Wheel
   odometry is merged (anchor held) up to its rate. Unit tests set both to `0`
   to stay deterministic (default-on otherwise). Validated on real data:
   BotanicGarden's ~400 Hz IMU + ~200 Hz wheels used to slow
   `estimated_navstate()` to ~174 ms average undecimated; MulRan DCC01's ~100 Hz
   Xsens IMU + the background optimizer thread now keeps `estimated_navstate()`
   at a **42 us average / 730 us max** over 5406 LIO queries with the 5 Hz cap
   on (the un-marginalized single ISAM2 graph still grows, plan 4.11 / Phase 10,
   but the cap + thread keep the query path fast). ALWAYS keep these on (or set
   `enable_optimizer_thread`) for any real high-rate IMU/wheel source.
   (A `gui_preview_sensors` GOTCHA found along the way too: a per-entry
   `enabled:` flag, not just the module-level one, is needed if a
   `dataset_input` module both declares `gui_preview_sensors` AND might run
   with `MOLA_WITH_GUI=false` -- otherwise the dataset module's GUI-updater
   thread throws "Could not find a running MolaViz module" on every sample.)

## Build / test

```bash
cd ~/ros2_ws
colcon build --packages-select mola_mapper_3d
colcon test  --packages-select mola_mapper_3d && colcon test-result --verbose
```

## Real-dataset runs

```bash
export KITTI_BASE_DIR=/path/to/kitti_root
export MOLA_ODOMETRY_PIPELINE_YAML=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml
KITTI_SEQ=04 MOLA_LINK_FIRST_POSE_SIGMA=1e-6 \
  mola-cli mola-cli-launchs/lidar_odometry_mapper3d_from_kitti.yaml
# MOLA_WITH_GUI=false for headless; MOLA_TIME_WARP=N to speed up/slow down.
```

MulRan launcher (`lidar_odometry_mapper3d_from_mulran.yaml`), RUN for real end
to end on DCC01 (2026-06-26, `MULRAN_BASE_DIR=.../MulRan MULRAN_SEQ=DCC01`,
headless, full ~550 s / 69262-message sequence). This is the first run
exercising LiDAR + IMU + **GNSS** together (KITTI/BotanicGarden have no GPS).
It self-contains `estimate_geo_reference: true` and
`link_first_pose_to_reference_origin_sigma: 1e-6` (needs both out of the box),
plus `enable_optimizer_thread: true` and `imu_max_insert_rate_hz: 5.0` so it
runs fast by default. Findings:
- **Query speed**: LIO's per-scan `estimated_navstate()` averaged **42 us**
  (max 730 us) over 5406 calls -- the >100 ms synchronous-solve problem is gone
  (optimizer thread + the 5 Hz IMU summarization that bounds graph growth).
- **Auto geo-referencing WORKS**: `T_enu_to_map` converges in a few seconds /
  ~10-23 GNSS factors; yaw locks to ~0.2-0.66 deg (the Xsens absolute-attitude
  `Pose3RotationFactor`), position sigma floors at ~1.1-1.5 m (GNSS noise).
- **Convergence criterion fix** (2026-06-26): the geo-ref convergence gate no
  longer includes the latest keyframe's ABSOLUTE position sigma. Unlike the
  smoother's sliding window, the central map pins a far gauge anchor, so that
  sigma grows with distance from it (GNSS-floored, a few meters) -- it is the
  expected DRIFT of odom_kf wrt ENU, not a geo-ref-quality signal, and gating on
  it prevented convergence from ever latching on long trajectories. The gate is
  now `enuPos <= conv_pos_sigma && max(enuOri, kfOri) <= conv_ori_sigma`
  (`Mapper3D_Fusion.cpp` `optimize_and_refresh`). MulRan's
  `convergence_max_position_sigma` was loosened 1.0 -> 1.5 m (its GNSS floors
  ~1.1 m). The `[geo-ref] convergence check: enu(...) kf(... [drift, not gated])`
  DEBUG trace shows the live sigmas + the growing kf drift.
- **Front-end reset no longer wipes the central map** (fixed 2026-06-26):
  `LidarOdometry` calls `navstate_fuse->reset()` once during its initial
  (re)localization (`LidarOdometry_InitialLocalization.cpp`). Mapper3D's
  `reset()` USED to do a full `state_.clear()` + `reinitialize_gtsam_locked()`,
  wiping the whole central map (keyframes + tentative geo origin) accumulated
  during LIO warmup (geo-ref converged, got wiped, re-converged -- CONVERGED
  announced twice). `reset()` now resets ONLY the short-term per-source
  integration anchors (wheel/IMU/ingestion chains) via
  `reset_sensor_anchors_locked()`; the keyframes, factor graph, geo-referencing,
  diagnostic counters and converged-announcement persist (the map is the
  shared, persistent world model and must survive one front end relocalizing).
  Verified on DCC01: tentative origin + CONVERGED now appear exactly once. See
  the YAML's own header comment and the `MULRAN_BASE_DIR`/`MULRAN_SEQ` env vars.
- **LIO's ICP no longer collapses to 0% after geo-ref convergence** (fixed
  2026-06-26): the short-term prediction `estimated_navstate(t, {odom})` LIO
  queries each scan used to be reconstructed globally as
  `X(kf) (-) T_map_to_odom`. The instant GNSS+IMU geo-ref converged and began
  pulling `{map}` (non-rigidly) away from LIO's `{odom}`, that reconstruction
  injected the accumulated/per-solve map correction into the odom-frame ICP
  initial guess -- meter/degree jumps between consecutive 30 ms scans -- so ICP
  diverged and stayed at 0% goodness for the rest of the run (DCC01: collapse
  ~3 s after CONVERGED, 1215 zero-goodness scans). Now the odom-frame prediction
  is frame-local (anchored on LIO's own last reported pose, twist-extrapolated;
  see the design note above). Re-validated on DCC01 (time-warp 10, 247k log
  lines): **0** zero-goodness scans, ICP goodness holds ~85% across the whole
  post-convergence trajectory, `Est.twist` populated with healthy `cov_inv`
  (motion model accepted, not discarded), 0 exceptions, geo-ref still converges
  identically (CONVERGED at the same point).

BotanicGarden launcher (`lidar_odometry_mapper3d_from_botanicgarden.yaml`),
RUN for real end to end:

```bash
export BOTANICGARDEN_LIO_BAG=$HOME/datasets/botanic/1018_00_LIO.bag
export MOLA_ODOMETRY_PIPELINE_YAML=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml
mola-cli mola-cli-launchs/lidar_odometry_mapper3d_from_botanicgarden.yaml
```

The BotanicGarden launcher is now SELF-CONTAINED (inline `params:` block):
the gauge anchor (`link_first_pose_to_reference_origin_sigma: 1e-6`), the
20 Hz IMU/wheel decimation, the background optimizer thread and a 20 Hz
high-rate publisher are baked in as defaults, so it runs out of the box with
just the two env vars above (all still overridable). It used to die a few
seconds in with `gtsam::IndeterminantLinearSystemException` (gauge-free graph)
when run without the magic env vars; that is fixed. All values remain
overridable, e.g. `MOLA_MAPPER3D_OPTIMIZER_THREAD=false` to force the
deterministic synchronous solve, `MOLA_MAPPER3D_IMU_MAX_RATE_HZ=...`, etc.

Reads a ROS 1 `.bag` directly (no ROS 1 install) via `mola::Rosbag1Dataset`
(`mola_input_rosbag1`); use that package's `rosbag1-info <bag>` CLI tool to
confirm a dataset's sensor inventory before wiring a new launcher (don't
trust a dataset's own README/AGENTS.md blindly -- BotanicGarden's actually
HAS a `/odom` wheel-odometry topic the dataset's own AGENTS.md note didn't
mention). This dataset has LiDAR + IMU + wheels, NO GNSS. See lesson 6 above
for the `MOLA_MAPPER3D_IMU_MAX_RATE_HZ`/`MOLA_MAPPER3D_ODOM_MAX_RATE_HZ` caps.

## Code style

clang-format-14; no one-line `if`; one variable per line; no em/en dashes;
American spelling; anonymous namespaces over `static`. clang-tidy per the repo
`.clang-tidy`. Don't sign commits as an AI agent. Keep this file and the plan in
sync with the code.
