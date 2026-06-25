# mola_mapper_3d

Central 3D SLAM map for the [MOLA](https://github.com/MOLAorg/mola) framework.

`mola_mapper_3d` holds **one global, optimized representation of the world**,
fusing multiple odometry sources (wheels, LiDAR-odometry, visual-odometry),
IMU and GNSS into a single keyframe-based map (a `mrpt::maps::CSimpleMap` with
raw observations and annotations) plus a GTSAM factor graph, with anytime loop
closure, geo-referencing, lifelong keyframe management and relocalization.

It provides, through `mola_kernel` interfaces:

- `mola::NavStateFilter`: the single source of truth for short-term pose
  prediction queried by LIO/VIO front ends (replaces
  `mola_state_estimation_{simple,smoother}` when in use).
- `mola::LocalizationSourceBase`: the fused vehicle localization.
- `mola::MapSourceBase`: the map and geo-referencing.
- `mola::DiagnosticsProvider`: structured diagnostics.

## Status

Implemented so far:
- Package scaffolding, build, module registration and loading.
- Central world-model state (keyframes, connectivity, geo-ref, GTSAM pimpl).
- Native `gtsam::ISAM2` fusion over all keyframes with the out-of-order guard.
- Multi-source odometry fusion (per-source `T_map_to_odom_i`), `fuse_pose`,
  `fuse_odometry` (+ decimation), `fuse_twist`.
- IMU: accelerometer gravity leveling, absolute attitude/azimuth, optional gyro.
- GNSS: `FactorGnssMapEnu` ENU factors (fixed or tentative geo-reference), Huber.
- `estimated_navstate()` (constant-velocity extrapolation + frame conversion).

Not yet implemented: live geo-ref *estimation* convergence/publication,
keyframe ingestion API, loop closure, map save/load,
async high-rate publisher, visualization, relocalization, huge-map
spatial paging.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select mola_mapper_3d
colcon test --packages-select mola_mapper_3d
```

## Layout

```
module/include/mola_mapper_3d/   Public headers (Mapper3D, Parameters, WorldModelState)
module/src/                      Implementation (split by concern)
apps/                            mola-mapper-3d-cli (offline front end)
params/                          Default YAML configuration
test/                            Unit tests
```

## License

GNU GPL v3. Closed-source licenses available upon request.
