# mola_mapper_3d - AI agent context guide

Central 3D SLAM map for MOLA: fuses LIO/VIO/IMU/GNSS/wheels into ONE optimized
world model (keyframes as a `CSimpleMap` + a GTSAM factor graph), with anytime
loop closure, geo-referencing, lifelong keyframe management and relocalization.

Full plan + task checklist (keep it in sync as you work):
`~/plans/900_mola_mapper_3d_plan.md`. Do NOT mention phase numbers in this repo
docs or code.

Use clang-format-14 on generated code.

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
  ImuGravityFilter.h             Robust low-dynamics gravity-direction reducer
                                 (now driven per-keyframe-interval; see design note)
module/src/
  Mapper3D.cpp                   Lifecycle: initialize/spinOnce/reset/diagnostics + IMPLEMENTS_MRPT_OBJECT
  Mapper3D_Fusion.cpp            Keyframe management + factor-graph fusion + estimated_navstate
  Mapper3D_KeyframeIngestion.cpp SharedKeyframeMap sink: requestInsertKeyframe() (anchor-once + consecutive chain)
  Mapper3D_GUI.cpp               Optional MolaViz/MolaVizImGui viz: KF tree + graph edges + per-source movable {odom_i} frames + the {enu} geo-ref frame marker (drawn at inverse(T_enu_to_map) while geo-referencing) + GuiWidgetDescription panel (Status / Geo-ref / View tabs: KF/edge/IMU-factor counts, geo-ref T_enu_to_map + GNSS factors + per-source T_map_to_odom drift as trans/rot, now the INSTANTANEOUS derived transform -- see the T_map_to_odom_i design note). The View tab has a "Viz reference frame" combo (map / enu) selecting the scene origin: it applies a `vizXform` (identity for map, `T_enu_to_map` for enu) to all scene containers + movable odom frames, so "enu" renders the map North-oriented (default; {enu} always exists as the identity weak-prior until geo-ref converges). An XYZ "enu" corner is drawn at the scene origin only while the enu frame is selected. NOTE: odom drift is ~0 on pure-odometry runs (no GNSS => {map} == LIO's {odom}); once GNSS/IMU geo-referencing pulls/levels the map the ROTATION grows to the real map-vs-odom angle (e.g. ~11 deg on DCC01) and the TRANSLATION is its lever-arm consequence over the trajectory (100s of m), NOT a sign the map is wrong (DCC01 map is ~6.7 m RMSE to GT).
  Mapper3D_SensorCallbacks.cpp   onNewObservation dispatch -> fuse_*()
  ImuGravityFilter.cpp           Robust low-dynamics gravity-direction reducer
                                  (pure/testable): given a window of (already
                                  lever-arm-corrected) accel/gyro samples, rejects
                                  motion-contaminated ones, robustly averages the
                                  survivors -> ONE gravity dir + data-earned sigma.
                                  fuse_imu feeds it per keyframe interval (the raw
                                  buffering/lever-arm is mola_imu_preintegration's
                                  ImuTransformer + LocalVelocityBuffer, reused)
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
  test-multisource-georef-stability.cpp  MulRan-style end-to-end sweep driven via
                                  onNewObservation()/requestInsertKeyframe(): a ~1 km flat
                                  trajectory with a systematic per-keyframe LIO pitch (tilt)
                                  bias, wheels in a FAR-offset frame, IMU accel+attitude, and
                                  noisy GNSS (0.1 m XY / 0.2 m Z). Sweeps source ON/OFF combos
                                  (LIO+IMUacc, +IMUatt, +GPS, +wheels, full) and asserts pose
                                  recovery, IMU leveling / attitude azimuth recovery,
                                  travel-direction consistency, and that T_enu_to_map stays
                                  BOUNDED (anti-regression for the DCC01 unbounded-growth
                                  report). NOTE: in this CLEAN synthetic world T_enu_to_map
                                  stays bounded (~10 m, never 100s of m) in ALL GNSS combos --
                                  the unbounded-growth failure is NOT reproduced by clean
                                  multi-source data, so it is real-data-specific (GNSS outliers /
                                  height jumps / larger or yaw drift / revisits). The LIO+GPS
                                  case uses centimeter (RTK) GNSS: it recovers the HORIZONTAL
                                  trajectory to <1 m; full-3D recovery is bounded but not cm
                                  because the LIO tilt drift distorts the trajectory SHAPE (it
                                  lives in the stiff relative-position edges, not removable by
                                  position fixes nor IMU attitude). This test drove the
                                  reinitialize_gtsam_locked T_enu_to_map prior fix (pin roll/pitch
                                  even when estimating geo-ref; see the IMU-gravity-leveling note).
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
- **IMU never creates keyframes nor inserts per-sample factors** (rewritten
  2026-06-29). `fuse_imu()` ONLY moves each raw reading to the vehicle frame with
  a per-`sensorLabel` `mola::imu::ImuTransformer` (rotation + rigid lever-arm /
  centripetal correction `a_body = R*a_imu - alpha x t - w x (w x t)`, removing
  the motion contamination the old code could only gate away) and pushes proper
  accel / angular velocity / absolute attitude into ONE global
  `mola::imu::LocalVelocityBuffer` (`imu_buffer_`, `max_time_window` >= the max
  keyframe gap so a full interval survives pruning). It adds NO factor and wakes
  no optimizer. The backend factors are built later, ONCE per real keyframe, by
  `emit_imu_factors_for_keyframe_locked()` (hooked at the tail of
  `create_or_get_keyframe_by_timestamp_locked()` so IMU rides WHATEVER source's
  keyframe cadence): it drains `window_since(prev_imu_kf_t, new_kf_t)` and emits
  the gravity/attitude/gyro factors below. So the IMU factor rate == the keyframe
  rate (no `imu_max_insert_rate_hz` summarization anymore; that param + the
  bespoke `imu_accum_` are deleted). The predictor side will read the same buffer
  (future work, plan 4.12). See `Mapper3D_Fusion.cpp::fuse_imu` /
  `ingest_imu_sample_locked` / `emit_imu_factors_for_keyframe_locked`.
- **High-rate wheels aggregation** (`aggregate_high_rate_into_edges`, opt-in;
  default false so unit tests are unchanged; true in the BotanicGarden launch):
  wheels no longer spawn a keyframe per sample. They share a bounded-rate
  keyframe clock (`sensor_keyframe_min_period`, default 0.5 s), and wheel
  odometry is aggregated into ONE relative-pose edge
  `Between(T(prev_kf), T(cur_kf))` per keyframe transition (the "consecutive
  frame edge" model; no `{odom_wheels}` frame variable created). Synthetic test:
  201 IMU+wheel samples -> ~8 keyframes with the trajectory recovered
  (`test-highrate-aggregation`). On BotanicGarden the wheels reuse LIO's denser
  scan keyframes, adding ~zero extra. See `Mapper3D_Fusion.cpp::fuse_odometry`.
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
- **The `{odom_i}` prediction extrapolates with a FINITE-DIFFERENCE twist from
  each source's consecutive raw poses (`RawSourcePose::local_twist`), NOT the
  graph `V(kf)`/`W(kf)`.** This is the TWIST half of the frame-local design (the
  pose ANCHOR being frame-local is only half the story): the graph twist is
  re-optimized every iSAM2 solve by the ABSOLUTE factors (GNSS / IMU-gravity
  leveling / loop closure), so it JITTERS independently of the source's true
  motion. That jitter became catastrophic once `T_enu_to_map` roll/pitch was
  pinned (so those factors bend the soft keyframe chain rather than tilt the
  transform): the latest keyframe's V/W swung per solve and -- because the
  prediction extrapolated the (stable) raw anchor with that (volatile) graph
  twist -- consecutive scans got wildly different motion priors, collapsing LIO's
  ICP to ~0% goodness on MulRan DCC01. Differencing the source's OWN raw poses
  gives a twist immune to every `{map}`-frame correction, so anchor AND twist are
  both frame-local. (Originally added as a plan-4.7 workaround for an IMU-induced
  graph V/W corruption -- V ~0.2 m/s vs ~10 true, W ~42 rad/s -- since fixed by
  "IMU on keyframes only"; kept because the absolute-factor jitter above is a
  permanent property of the non-windowed central map, not that one bug.) Trace
  with `MOLA_MAPPER3D_TRACE_PREDICT=1`.
- **IMU gravity leveling (why a per-sample accelerometer factor does NOT level
  the map, and the filtered-gravity fix).** A single accelerometer sample
  measures specific force = gravity + body acceleration, so its "up" is
  contaminated by vehicle dynamics. Diagnosed on MulRan DCC01 (GPS off, IMU on,
  geo-ref off) with `MOLA_MAPPER3D_TRACE_IMU=1`: the per-sample
  `MeasuredGravityFactor` residuals were ~2.2 deg RMS (tail to >12 deg) and
  RANDOM -- their mean residual VECTOR was ~0.07 deg, i.e. zero-mean motion
  noise, NOT a systematic map tilt. A zero-mean observation can only pin the
  map's MEAN tilt to ~noise/sqrt(N) (~0.08 deg here) and never bends the stiff
  LIO orientation chain, so the slow z-drift (sub-degree pitch integrated over
  km) survived. The current architecture (rewritten 2026-06-29) addresses each:
  (1) the leveling factor is built ONCE per keyframe by
  `emit_imu_factors_for_keyframe_locked()`, NOT per sample: it drains the
  `LocalVelocityBuffer` interval `(prev_imu_kf, new_kf]`, runs the robust
  `ImuGravityFilter` reduce (now a STATELESS per-interval reducer: rejects
  samples with `||a||` far from g or `||w||` above `imu_gravity_gyro_tol_deg`,
  robustly averages the survivors) and emits ONE `MeasuredGravityFactor` with a
  DATA-EARNED sigma (angular spread / sqrt(n), clamped). Because the accel was
  lever-arm-corrected by `ImuTransformer` at ingest, the factor uses an IDENTITY
  sensor pose and the centripetal/tangential contamination is SUBTRACTED, not
  just gated. `imu_gravity_min_samples` (default 10) gates a window; on dense
  synthetic keyframes (~1 sample/interval) lower it to 1.
  (2) the absolute-attitude `Pose3RotationFactor` is now added ALWAYS when the
  IMU reports an orientation quaternion, REGARDLESS of geo-ref. The absolute
  attitude IS an azimuth reference, so feeding it lets iSAM2 AUTO-ESTIMATE the
  geo-reference yaw even with GNSS off. To make that land on `T_enu_to_map`
  (azimuth) instead of fighting the leveled {map}, the F0 prior is now
  ANISOTROPIC: tight on roll/pitch (so gravity / GNSS still level the KEYFRAMES,
  not F0) but WEAK on yaw (so IMU azimuth drives F0.yaw). Translation is tight
  ONLY for pure odometry (map origin == ENU origin); when ESTIMATING geo-ref the
  translation stays WEAK (it is the unknown geo offset). **This roll/pitch pin
  applies whether or not geo-ref is being estimated** (`reinitialize_gtsam_locked`,
  the `!fixed_geo_reference` branch): ENU and {map} are BOTH gravity-aligned, so
  `T_enu_to_map` is a level yaw+translation, and a FREE roll/pitch on it lets
  GNSS satisfy itself for free by TILTING the weakly-prior'd transform instead
  of flattening the keyframes -- leaving {map} z/tilt-drifted and GNSS unable to
  correct it (it previously fell through to an all-weak isotropic prior when
  estimating geo-ref). Validated on DCC01 IMU-only (GNSS off,
  `estimate_geo_reference:false`): F0 yaw auto-locked to -170.3 deg, within ~4
  deg of the GNSS-derived -174.3 deg; roll/pitch stayed 0. Synthetic LIO+GPS
  (no IMU, `test-multisource-georef-stability`): the pin takes the {map} z/tilt
  drift from ~40 m to ~2.5 m even with GNSS position only.
  (3) the LIO odometry between-edge chain is near-rigid in roll/pitch (data-driven
  sigma + a 1e-3 deg floor), so even a clean gravity factor cannot bend it --
  `odometry_edge_min_sigma_rollpitch_deg` (>0) RAISES only the roll/pitch floor,
  leaving yaw stiff (no yaw gauge loss). The MulRan launcher sets it to 3.0 (env
  `MOLA_MAPPER3D_ODOM_RP_SIGMA`). Measured on DCC01 IMU-only after the rewrite:
  gravity-resid mean 1.1 deg, mean-VECTOR-norm **0.24 deg** (random, not a
  systematic tilt), keyframe path-tilt **0.46 deg** with graph z-span halving the
  raw GICP-LIO drift (76 m -> 42 m). Real IMU preintegration + accel bias (plan
  4.12) remains the principled end-state (it would subtract modeled body accel
  instead of gating it away).
  **Diagnostic:** `MOLA_MAPPER3D_TRACE_IMU=1` logs F0's roll/pitch/yaw and the
  gravity-residual distribution (mean/median/p90/max + the mean residual VECTOR
  norm: ~mean => systematic tilt, <<mean => random motion noise) each solve
  (`Mapper3D::trace_imu_factors_locked`).
- **The gravity-factor RESIDUAL is NOT a "the map is level" metric (measured).**
  A low residual only proves the keyframes align to the MEASURED gravity, not to
  true vertical. Measured on DCC01 IMU-only (GPS off, geo-ref off,
  `MOLA_MAPPER3D_TRACE_GEOM=1`, `Mapper3D::trace_keyframe_geometry_locked`): the
  gravity residual sits at ~0.5 deg (factor satisfied) while the {map} keyframe
  PATH still climbs ~29 m in z over ~1.7 km (GT z-span ~2 m), a persistent
  ~+1.1 deg SYSTEMATIC keyframe pitch / ~1 deg path tilt. Decomposition from the
  trace: (1) the factor IS working -- it roughly HALVES the raw LIO z-drift
  (RAW-LIO odom z-span ~58 m -> graph ~29 m); (2) but it cannot flatten the rest,
  because orientation-only leveling fights the stiff translation chain AND the
  FILTERED GRAVITY REFERENCE itself carries a ~0.6-0.7 deg systematic lean in the
  VEHICLE frame (`[GRAV-BODY-TRACE]` mean up_vehicle ~ (-0.012, 0.011, 1.0)) --
  an accelerometer-bias / unmodeled-IMU-mount tilt (MulRan's `imuPoseOnVehicle_`
  is translation-only, NO rotation) that the factor faithfully levels TO. So a
  sub-degree gravity-reference bias integrates into tens of meters of z-drift,
  and without an absolute z reference (GNSS off) it cannot be removed by leveling
  alone. This is the real reason the visualizer still shows a tilted path; the
  separate, LARGER tilt of the LIO LOCAL MAP is LIO's own {odom} z-drift (~58 m),
  which mapper3d deliberately does NOT correct (frame-local prediction design).
  The principled fix is plan 4.12 (IMU preintegration WITH accel-bias
  estimation), which would estimate+subtract the bias instead of baking it into
  "gravity." **Diagnostic:** `MOLA_MAPPER3D_TRACE_GEOM=1` logs, each solve, the
  keyframe-path z-span + z-vs-arc slope (apparent path tilt), the mean SIGNED
  keyframe pitch/roll, the raw-LIO odom z-span (to tell "factor helping" from
  "factor outvoted"), and the running-mean filtered gravity direction in the
  vehicle frame (`[GRAV-BODY-TRACE]`, the systematic reference lean).
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

## Design ideas (planned rewire: geo-ref robustness + kinematics)

These are agreed design directions, not all implemented yet. Keep this in sync
as the rewire lands.

- **Two-phase geo-ref estimation (GNC bootstrap, then iSAM2 + robust).** A single
  rigid `T_enu_to_map` plus a Huber kernel inside the live iSAM2 graph is a poor
  bootstrap: when only a FEW GNSS fixes exist and the geo-ref is not yet
  established, the joint problem is ill-conditioned and Huber DOWNWEIGHTS exactly
  the large residuals needed to pull the transform toward the true basin, so it
  can wander (the DCC01 unbounded-growth report). Plan:
  1. While geo-ref is NOT established, do NOT add GNSS factors to the live iSAM2
     graph. iSAM2 runs LIO (+IMU) only.
  2. Collect `(T(kf) in {map}, observed ENU point)` correspondences. Run a
     SEPARATE, small GNC optimization (`gtsam::GncOptimizer`, e.g. TLS) over the
     single unknown `T_enu_to_map` (keyframe poses fixed at their current iSAM2
     estimate) using the `FactorGnssMapEnu` factors with NON-robust base noise
     models. GNC's graduated non-convexity escapes bad local minima and rejects
     outliers more reliably than a fixed Huber threshold during bootstrap.
  3. Once GNC converges with a tight covariance, LATCH `geo_reference` and from
     then on add GNSS factors to the live iSAM2 graph WITH a robust (Huber)
     kernel for ongoing per-fix outlier rejection.
- **Undefined-geo-ref guard (do not estimate until observable).** Do not even
  attempt the GNC bootstrap until the SPAN of the collected ENU points is
  significant relative to their own noise, i.e. `span > N * sigma_enu` for some
  N (a few). A short cluster of fixes (vehicle ~stationary, or only a couple of
  fixes) does NOT determine the geo-ref yaw / plane and must be treated as an
  UNDEFINED situation: keep `T_enu_to_map` at its prior and publish no geo-ref.
  Roll/pitch of the geo-ref are only observable from a CURVED (non-degenerate)
  trajectory or from IMU gravity; a straight-line GNSS track leaves roll free.
- **Kinematics factors are distinct from odometry BetweenFactors.** SE(3)
  relative-pose `Between` edges along consecutive keyframes encode ONE odometry
  source's measured increment (e.g. LIO or VIO). KINEMATIC factors
  (`add_kinematic_factors`: constant-velocity / tricycle) should additionally
  link EVERY pair of consecutive keyframes regardless of which source created
  them, so poses from different odometry sources are merged via the shared
  kinematic/twist model, and the body twist `V(kf)/W(kf)` is estimated even with
  a single source (they act as a smoothing / filtering prior and the twist
  estimator). Kinematic factors must be SOFT (loose enough to allow non-constant
  velocity and not prevent long trajectories from being blended by GNSS/IMU) and
  are NEVER robustified (unlike GNSS).

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

6. **Wheel odometry is rate-capped (`odometry_max_insert_rate_hz`, default
   `5.0` Hz; `0` = insert every reading); the IMU no longer needs a cap.** The
   old `imu_max_insert_rate_hz` summarization was DELETED in the 2026-06-29
   `fuse_imu` rewrite: the IMU now creates no keyframes and inserts no per-sample
   factors, so its factor rate is intrinsically bounded by the keyframe rate (one
   gravity/attitude/gyro set per real keyframe, drained from the
   `LocalVelocityBuffer`). Wheel odometry is still merged (anchor held) up to its
   rate. Unit tests set the wheel cap to `0` to stay deterministic. Validated on
   real data: MulRan DCC01's ~100 Hz Xsens IMU + the background optimizer thread
   kept `estimated_navstate()` at a **42 us average / 730 us max** over 5406 LIO
   queries (the un-marginalized single ISAM2 graph still grows, plan 4.11 /
   Phase 10, but the keyframe-rate IMU factors + thread keep the query path fast).
   ALWAYS keep `enable_optimizer_thread` on for any real high-rate source.
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
plus `enable_optimizer_thread: true` so it runs fast by default. Findings:
- **Query speed**: LIO's per-scan `estimated_navstate()` averaged **42 us**
  (max 730 us) over 5406 calls -- the >100 ms synchronous-solve problem is gone
  (optimizer thread + IMU factors now emitted only at the keyframe rate).
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

## Call graph

`docs/call-graph.md` contains a Mermaid diagram tracing every major public-API
method down to its internal helpers, state stores, and the background optimizer.
**Keep it in sync with the code**: update it whenever you add, rename, or
significantly rewire any of the following:

- A public method in `Mapper3D.h` (interface implementations or extensions)
- The call chain inside `fuse_pose_locked()`, `request_insert_keyframe_locked()`,
  `optimize_and_refresh()`, or `estimated_navstate()`
- A new helper that sits on the hot path (e.g. a new `*_locked()` function called
  from multiple fusion paths)
- A change in the `sensor_kf_creation_allowed()` gate logic
- A change in the locking model (which mutex covers which call)

The table at the bottom of the diagram ("Key invariants") and the
"SharedMapOnly vs Auto mode" section must also be updated if the locking model
or gating logic changes.

## Code style

clang-format-14; no one-line `if`; one variable per line; no em/en dashes;
American spelling; anonymous namespaces over `static`. clang-tidy per the repo
`.clang-tidy`. Don't sign commits as an AI agent. Keep this file, the plan, and
`docs/call-graph.md` in sync with the code.
