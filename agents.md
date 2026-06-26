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
  Mapper3D_GUI.cpp               Optional MolaViz/MolaVizImGui viz: KF tree + graph edges + per-source movable {odom_i} frames + the {enu} geo-ref frame marker (drawn at inverse(T_enu_to_map) while geo-referencing) + GuiWidgetDescription panel (Status / Geo-ref / View tabs: KF/edge/IMU-factor counts, geo-ref T_enu_to_map + GNSS factors + per-source T_map_to_odom drift as trans/rot). NOTE: odom drift is ~0 on pure-odometry runs (no GNSS => {map} == LIO's {odom}); it grows (e.g. ~19 m / ~5 deg on DCC01) only once GNSS/IMU geo-referencing pulls the map.
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
  source, plus `{map}` and `{enu}`; the graph estimates `T_map_to_odom_i` and
  `T_enu_to_map`. Reuse `mola_gtsam_factors` for kinematic/IMU/GNSS factors.
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
- `SharedKeyframeMap::requestInsertKeyframe()` (the "consecutive-frame edge"
  drift-fix, plan section 2.8): the first request from a given
  `source_frame_id` anchors `F(source)` with a tight `Between()`; every later
  request instead adds a tight `Between(T(prev_kf), T(kf))` using the front
  end's own relative motion. Both ALWAYS use Mapper3D's own configured
  `keyframe_ingestion_sigma_lin/ang_deg` noise, never the request's own
  `pose_in_source` covariance (see lessons below for why).

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
2. **Never trust a `pose_in_source`/`fuse_pose()` input covariance directly**
   for the noise model. Real front ends report pathological values (LIO's
   `FixedPose` relocalization seed pins `cov` at `1e-12`); mixed with weak
   priors elsewhere in the graph, that ill-conditions iSAM2's Cholesky. Use
   your own configured sigma, and/or floor the input (see
   `MIN_POSE_SIGMA_LIN/ANG` in `Mapper3D_Fusion.cpp`).
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
