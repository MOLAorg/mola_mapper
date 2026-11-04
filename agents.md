# mola_mapper - AI agent context guide

The global back end of a MOLA system: a container next to a front end (LIO, later VO) and
its state estimator (`mola_state_estimation_smoother`), adding a central keyframe map, loop
closure detection and a global pose graph. It does little estimation of its own.

Plan and task checklist: `~/plans/900_mola_mapper.md` (keep in sync; no plan section numbers
in this repo). Use clang-format-14.

## Topology

Separate MOLA modules, composed by `mola-cli-launchs/*.yaml`:
- LIO (`mola::LidarOdometry`) keeps the smoother as its only `NavStateFilter`, and pushes
  sparse keyframes to the mapper through `mola::SharedKeyframeMap` (source frame
  `lidar_odom_kf`).
- The smoother fuses LIO increments (and, in "owner" mode, the mapper's absolute `{map}`
  corrections). Optionally it tracks IMU biases (`MOLA_NAVSTATE_IMU_BIAS_TRACKING`) and
  offers them as `mola::ImuBiasSource`.
- `mola::mapper::Mapper` (ExecutableBase, RawDataConsumer, MapSourceBase,
  DiagnosticsProvider, SharedKeyframeMap) is NOT a NavStateFilter.

The mapper owns GNSS and the geo-reference: in owner mode the smoother runs without GNSS and
without absolute attitude, and gets the optimized newest keyframe pose with
`fuse_pose(t, pose, "map")` (`feedback_to_state_estimator`). The smoother's
`set_geo_reference()` is not used mid-run (it resets the smoother).

## Layout

```
module/include/mola_mapper/
  Mapper.h          the module (keyframe sink, raw data, threads, outputs)
  GlobalGraph.h     optimize_global_graph(): pure function, GTSAM batch LM + Marginals
  GlobalGraphIO.h   save/load the graph input (offline re-optimization), TUM, georef
  GyroIntegration.h integrate_gyro(): rotation between keyframes from gyro readings
  Parameters.h      Parameters (module) and GlobalGraphParams (graph)
module/src/         Mapper.cpp, Mapper_LoopClosure.cpp, GlobalGraph*.cpp, ...
apps/mola-mapper-reoptimize.cpp   re-solves a saved graph input with other parameters
params/mapper.yaml                every setting with a MOLA_MAPPER_* environment hook
params/loop-closure-f2f-mapper*.yaml  mola_sm_loop_closure detector pipelines
mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml  rosbag2 -> LIO + smoother + mapper
mola-cli-launchs/lio_smoother_mapper_from_kitti.yaml    KITTI odometry (params/mapper-kitti.yaml)
```

## Keyframes and attached data

- `requestInsertKeyframe()`: rejects keyframes not newer than the last; initial guess = newest
  estimate + front end increment.
- GNSS (`gnss_sensor_labels_re`): attached to the nearest keyframe within
  `gnss_max_time_offset`, only once a keyframe at or after the reading exists (pending queue).
- Gravity (`imu_sensor_labels_re`): accelerometer averaged over `imu_gravity_window` before
  each keyframe, stored as a CObservationIMU in the keyframe aux frame.
- Gyro (`gyro_sensor_labels_re`, first matching label): readings integrated between
  consecutive keyframes (`integrate_gyro()`, exact tick-based durations), bias from the first
  `ImuBiasSource` with an estimate (else 0 with `gyro_bias_sigma`), rotated to the vehicle
  frame. Sigma: noise density * sqrt(T), bias sigma * T, scale sigma * accumulated angle.

## Global graph (GlobalGraph.cpp)

Keys `P(i)` (keyframe poses in `{map}`), `T(0)` (T_enu_to_map). Factors:
- odometry chain (same source frame): stiff by default, sigma = floor + per sqrt(m) + per
  radian turned (`odometry_noise_ang_per_rad`: LIO heading errors concentrate in turns);
  different source frames are joined by a weak edge;
- loop closures: Huber, with sigma floors (the detector ICP covariance is overconfident);
- GNSS: `FactorGnssMapEnu` with a selectable robust kernel (default Cauchy);
- gravity: `mola_georeferencing::add_imu_gravity_factors`;
- gyro: `BetweenFactor<Pose3>` with free translation (sigma 1e3 m);
- weak prior on `T(0)`, prior on `P(0)` (gauge).

## Threads

- Optimizer thread: re-solves after new data, at most every `optimizer_min_period`.
- Loop closure thread: `mola_sm_loop_closure` `analyze()` (detection only) over the keyframe
  map, incremental scans plus a full one every `loop_closure_full_scan_every_n`; new edges
  merged in order. `finalize()` (also from `onQuit()`) runs extra full scans and solves.
- `state_mtx_` guards keyframes, loops and pending data; never call other modules with it held.

## Results (husky_2026-11-03_20-42-07_loop, u-blox GNSS only, F9P RTK as ground truth)

Owner mode, absolute horizontal RMSE: 5.4 m initially; 4.6 m with smoother bias tracking and
the per-radian odometry sigma; 4.35 m adding gyro factors (HandsFree or Ouster IMU, similar).
Gyro factors mostly cut p90/max and the revisit error. LIO alone (Ouster OS0) drifts ~11 m.

KITTI 00 (3.7 km, loop closures only): LIO APE RMSE 6.85 m, mapper 2.21 m (50 loops).

## Tests

`test/`: test-global-graph (loop closure, GNSS georef, gyro vs turn heading errors),
test-gyro-integration, test-lc-wait-idle.
