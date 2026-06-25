# mola_mapper_3d - AI agent context guide

Central 3D SLAM map for MOLA: fuses LIO/VIO/IMU/GNSS/wheels into ONE optimized
world model (keyframes as a `CSimpleMap` + a GTSAM factor graph), with anytime
loop closure, geo-referencing, lifelong keyframe management and relocalization.

Full plan + task checklist (keep it in sync as you work):
`~/plans/900_mola_mapper_3d_plan.md`. Do NOT mention phase numbers in this repo
docs or code.

## What it is

`mola::mapper_3d::Mapper3D` implements `mola::NavStateFilter`,
`LocalizationSourceBase`, `MapSourceBase`, and `DiagnosticsProvider`. It is the
single source of truth for short-term pose prediction that LIO/VIO query
(replacing `mola_state_estimation_{simple,smoother}` when used). It is NOT a
`FrontEndBase`: it does no raw-scan ICP; front ends feed it via `fuse_pose()`
and (later) a keyframe-insertion API.

## Layout

```
module/include/mola_mapper_3d/   Public headers
  Mapper3D.h                     Main class
  Parameters.h                   YAML-loaded config (navstate group mirrors the smoother)
  WorldModelState.h              Central map state (keyframes, connectivity, geo-ref, GTSAM pimpl)
module/src/
  Mapper3D.cpp                   Lifecycle: initialize/spinOnce/reset/diagnostics + IMPLEMENTS_MRPT_OBJECT
  Mapper3D_Fusion.cpp            Keyframe management + (WIP) factor-graph fusion + estimated_navstate
  Mapper3D_SensorCallbacks.cpp   onNewObservation dispatch -> fuse_*()
  WorldModelState.cpp            GtsamData pimpl (ISAM2/Values/NonlinearFactorGraph) + map helpers
  Parameters.cpp                 loadFrom(yaml)
  register.cpp                   MOLA_REGISTER_MODULE(mola::mapper_3d::Mapper3D)
apps/mola-mapper-3d-cli.cpp      Offline front end (skeleton)
params/mapper-3d.yaml            Default config
test/                            Unit tests (plain main() + MRPT ASSERT_ macros, run by mola_add_test)
```

## Key design notes (and why)

- GTSAM is hidden behind `WorldModelState::GtsamData` (pimpl via `mrpt::make_impl`)
  to keep it out of the public header. The pimpl must stay **copyable**, so use
  `std::optional<gtsam::ISAM2>`, not `unique_ptr`.
- Solver: full `ISAM2` over ALL keyframes (the central map, not a fixed-lag
  window) + LM on loop closure + a lightweight predictor for short-term queries.
  Persistent variables (`T_enu_to_map`, `T_map_to_odom_i`) must stay OUT of any
  lossy fixed-lag marginalization.
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

## Build / test

```bash
cd ~/ros2_ws
colcon build --packages-select mola_mapper_3d
colcon test  --packages-select mola_mapper_3d && colcon test-result --verbose
```

## Code style

clang-format-14; no one-line `if`; one variable per line; no em/en dashes;
American spelling; anonymous namespaces over `static`. clang-tidy per the repo
`.clang-tidy`. Don't sign commits as an AI agent. Keep this file and the plan in
sync with the code.
