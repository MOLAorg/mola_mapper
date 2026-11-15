# mola_mapper - AI agent context guide

The global back end of a MOLA system: a container next to a front end (LIO, later VO) and
its state estimator (`mola_state_estimation_smoother`), adding a central keyframe map, loop
closure detection and a global pose graph. It does little estimation of its own.

Use clang-format-14.

## Topology

Separate MOLA modules, composed by `mola-cli-launchs/*.yaml`:
- LIO (`mola::LidarOdometry`) keeps the smoother as its only `NavStateFilter`, and pushes
  sparse keyframes to the mapper through `mola::SharedKeyframeMap` (source frame
  `lidar_odom_kf`).
- The smoother is the live estimator: it fuses LIO increments and, optionally, GNSS with
  its own online geo-reference. Optionally it tracks IMU biases
  (`MOLA_NAVSTATE_IMU_BIAS_TRACKING`) and offers them as `mola::ImuBiasSource`.
- `mola::mapper::Mapper` (ExecutableBase, RawDataConsumer, MapSourceBase,
  DiagnosticsProvider, SharedKeyframeMap, LocalMapCorrectionSource) is NOT a NavStateFilter.
  After solves with new loops it advertises a `LocalMapCorrection`; LIO then pulls
  the keyframes around its newest one (ids are indices in `keyframes_`, poses
  relative to it) and rebuilds its local map.

No estimate flows from the mapper into the smoother. Both may take the same GNSS readings:
the smoother for a live geo-referenced estimate, the mapper for the geo-reference of its
global map (estimated in its own graph). The smoother's `set_geo_reference()` is not used
mid-run (it resets the smoother).

## Layout

```
module/include/mola_mapper/
  Mapper.h          the module (keyframe sink, raw data, threads, outputs)
  GlobalGraph.h     optimize_global_graph(): pure function, GTSAM batch LM + Marginals
  GlobalGraphIO.h   save/load the graph input (offline re-optimization), TUM, georef
  GyroIntegration.h integrate_gyro(): rotation between keyframes from gyro readings
  WheelOdometry.h   wheel_odometry_increment(): wheel odometry between keyframes (interpolated)
  GraphVisualization.h  build_graph_visualization(): the GUI scene of the keyframe graph
  Parameters.h      Parameters (module) and GlobalGraphParams (graph)
  MapperInputLog.h  record (record_input, background writer) / read the mapper input
  MapperReplay.h    replay_mapper_input(): feeds a thread-less Mapper, scans and solves
                    at fixed periods of data time (bit-exact with one TBB thread)
module/src/         Mapper.cpp, Mapper_LoopClosure.cpp, Mapper_GUI.cpp, GlobalGraph*.cpp, ...
apps/mola-mapper-reoptimize.cpp   re-solves a saved graph input with other parameters
apps/mola-mapper-cli.cpp          replays a recorded mapper input (tuning, reproducible runs)
params/mapper.yaml                every setting with a MOLA_MAPPER_* environment hook
params/loop-closure-f2f-mapper*.yaml  mola_sm_loop_closure detector pipelines
mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml  rosbag2 -> LIO + smoother + mapper
mola-cli-launchs/lio_smoother_mapper_from_kitti.yaml    KITTI odometry (params/mapper-kitti.yaml)
```

## Keyframes and attached data

- `requestInsertKeyframe()`: rejects keyframes not newer than the last; initial guess = newest
  estimate + front end increment.
- GNSS (`gnss_sensor_labels_re`, readings with GGA only): attached to the nearest keyframe
  within `gnss_max_time_offset`, only once a keyframe at or after the reading exists (pending
  queue). No time interpolation: the fix is applied at the keyframe pose.
- Images (`image_sensor_labels_re`): same nearest-keyframe rule as GNSS, per camera
  (`Keyframe::nearest`, slot "image:<label>"); written to `<output_simplemap>_Images/`
  (jpg/png) once final, by the `disk_io_` thread (waited for in `finalize()`); the
  keyframe keeps a lazy-load copy (`CImage` copies are shallow: never externalize the
  shared input observation). Not recorded, not in the graph input.
- Gravity (`imu_sensor_labels_re`, first matching label): accelerometer averaged over
  `imu_gravity_window` before each keyframe, stored as a CObservationIMU in the keyframe aux frame.
- Gyro (`gyro_sensor_labels_re`, first matching label): readings integrated between
  consecutive keyframes (`integrate_gyro()`, exact tick-based durations), bias from the first
  `ImuBiasSource` with an estimate (else 0 with `gyro_bias_sigma`), rotated to the vehicle
  frame. Sigma: noise density * sqrt(T), bias sigma * T, scale sigma * accumulated angle. The
  bias error is modeled as independent per factor, though it is common to all of them.
- Wheel odometry (`wheel_odom_sensor_labels_re`, `CObservationOdometry`, first matching label):
  increment between consecutive keyframes, interpolated at both stamps (gaps over
  `wheel_odom_max_gap`: none); keyframes the readings do not reach yet wait, in order
  (`pending_wheel_odom_`). Recorded; saved as `<prefix>.wheel_odom.txt` with the graph input.
- The bias sources are queried before taking `state_mtx_`.

## Global graph (GlobalGraph.cpp)

Keys `P(i)` (keyframe poses in `{map}`), `T(0)` (T_enu_to_map). Factors:
- odometry chain (same source frame): stiff by default, sigma = floor + per sqrt(m) + per
  radian turned (`odometry_noise_ang_per_rad`: LIO heading errors concentrate in turns);
  different source frames are joined by a weak edge;
- loop closures: Huber, with sigma floors (the detector ICP covariance is overconfident);
- GNSS: `FactorGnssMapEnu` with a selectable robust kernel (default Cauchy);
- gravity: `mola_georeferencing::add_imu_gravity_factors`;
- gyro: `BetweenFactor<Pose3>` with free translation (sigma 1e3 m);
- wheel odometry (if attached; `use_wheel_odometry`): forward and lateral translation of the
  increment in the frame of the first keyframe (`ExpressionFactor<Point3>`, vertical free),
  Gaussian by default; there, the chain edge forward component moves to a factor of its own
  with `wheel_odom_chain_robust_kernel` (default Cauchy 1.0): a LIO slip along a degenerate
  direction then gives way to the wheels, while the rest of the edge stays stiff. A robust kernel
  on the wheel factors instead (`wheel_odom_robust_kernel`) rejects wheel slip, but also the
  wheels when LIO is the one wrong: nothing in the graph tells them apart, except GNSS.
  Increments whose yaw disagrees with the gyroscope (`wheel_odom_max_yaw_error_*`) or shorter
  than 0.95 of the LIO forward motion along the direction of travel
  (`wheel_odom_min_ratio_to_front_end`, minus 2 cm) are dropped: encoder count
  losses read false turns and less motion, a degenerate LIO falls short, never beyond (wheel spin
  is the case left);
- weak prior on `T(0)`, prior on `P(0)` (gauge).
The geo-reference is returned (published, saved) only once observable: `georef_min_gnss_factors`
and a `T(0)` azimuth sigma below `georef_max_sigma_yaw_deg`.

## Threads

- Optimizer thread: re-solves after new data, at most every `optimizer_min_period`.
- Loop closure thread: `mola_sm_loop_closure` `analyze()` (detection only) over the keyframe
  map, incremental scans plus a full one every `loop_closure_full_scan_every_n`; new edges
  merged in order, except those contradicting the odometry chain between their keyframes
  (`check_loop_against_odometry()`: floor + fraction of the distance traveled; only within one
  source frame, so a front end that restarts must use a new `source_frame_id`), then those
  inconsistent with most of their neighbor loops (`LoopConsistencyGate`: cycles of two loops and
  the odometry between their ends; each group of neighbor loops is decided again from scratch
  when a loop joins it, off `state_mtx_`, so loops can be dropped and accepted again; `loops_`
  is the accepted ones). Proposed and odometry-rejected pairs (`loop_pairs_`,
  `rejected_loop_pairs_`) are excluded from later scans (exact
  keyframe pairs: nearby pairs remain candidates). One scan at a
  time (`lc_scan_mtx_`). `finalize()` (also from `onQuit()`)
  runs extra full scans and solves; each step and output is guarded, so one failure does not
  lose the other outputs.
- `state_mtx_` guards keyframes, loops and pending data; never call other modules with it held.
- GUI (`Mapper_GUI.cpp`, only if a `VizInterface` module exists): `spinOnce()` (launcher thread,
  `execution_rate`) redraws `mapper/graph` (keyframe spheres, odometry edges, accepted and
  optionally rejected loops as `CSetOfLines` with their own widths) when the graph changed,
  copying it under `state_mtx_` and building the scene off it, at most every
  `visualization.update_period`. It also moves the movable viz frame `<f>` of each front end
  whose keyframes come as `<f>_kf` (LIO draws in `publish_reference_frame`) to the optimized
  `T_map_<f>`, so LIO's local map and vehicle stay aligned with the corrected graph.
  Its redraw key: keyframes, solves, loop revision and rejected counts. Known cost: one
  `CSphere` per keyframe (fine up to a few thousand; very long sessions may need fewer).
- Limitations: one front end chain (keyframes of interleaved sources only get weak edges);
  all keyframe observations stay in RAM; every solve is a full batch.

## Reference results

- 1.3 km ground robot loop, low-cost GNSS only (RTK as ground truth), measured with a feedback
  of the mapper poses into the smoother since removed: absolute horizontal RMSE 5.4 m initially; 4.6 m with smoother bias tracking and
  the per-radian odometry sigma; 4.35 m adding gyro factors. LIO alone drifts ~11 m.
- KITTI 00 (3.7 km, loop closures only): LIO APE RMSE 6.85 m, mapper 2.21 m (50 loops).

## Tests

`test/`: test-global-graph (loop closure, GNSS georef and its observability, gyro vs turn
heading errors), test-gyro-integration, test-mapper-ingestion (GNSS/gravity/gyro attachment,
outputs), test-loop-odometry-check, test-loop-consistency, test-lc-wait-idle,
test-graph-visualization (GUI scene, no visualizer needed).
