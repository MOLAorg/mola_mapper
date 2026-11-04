[![CI Build colcon](https://github.com/MOLAorg/mola_mapper/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper/actions/workflows/build-ros.yml)
[![CI clang-format](https://github.com/MOLAorg/mola_mapper/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper/actions/workflows/check-clang-format.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/mola_mapper.html)
[![codecov](https://codecov.io/gh/MOLAorg/mola_mapper/graph/badge.svg?token=C11VFFK0NW)](https://codecov.io/gh/MOLAorg/mola_mapper)

# mola_mapper

Global back end of a [MOLA](https://github.com/MOLAorg/mola) system, compatible
with ROS 2.

`mola::mapper::Mapper` is a MOLA module that runs next to a front end (LiDAR
odometry) and its state estimator. It keeps a central map of keyframes, looks
for loop closures, and optimizes a global pose graph with GNSS, gravity and
gyroscope factors, which also yields the geo-reference. It does little
estimation of its own and is not a `mola::NavStateFilter`.

## Module topology

- `mola::LidarOdometry` (LIO) uses the state estimator
  (`mola::state_estimation_smoother::StateEstimationSmoother`) as its only
  `NavStateFilter`, and pushes sparse keyframes to the mapper through
  `mola::SharedKeyframeMap`.
- `mola::mapper::Mapper` receives those keyframes, and GNSS and IMU readings
  from the raw data source as a `mola::RawDataConsumer`. It also implements
  `mola::MapSourceBase` (geo-reference), `mola::DiagnosticsProvider` and
  `mola::ExecutableBase`.
- Optionally (`feedback_to_state_estimator`), the optimized pose of the newest
  keyframe is fused into the state estimator as an absolute pose in the map
  frame.
- After loop closures, it tells the front ends (`mola::LocalMapCorrectionSource`),
  which can then rebuild their local maps from the corrected keyframes around
  them, including those of earlier visits.

See [docs/call-graph.md](docs/call-graph.md) for the threads and call flow.

## Running with a rosbag2

`mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml` composes LIO, the
state estimator, the mapper and a rosbag2 dataset reader:

```bash
export MOLA_INPUT_ROSBAG2=/path/to/bag
export MOLA_LIDAR_TOPIC=/ouster/points
export MOLA_IMU_TOPIC=/ouster/imu
export MOLA_GNSS_TOPIC=/gps/fix          # optional
export MOLA_MAPPER_OUTPUT_TUM=/tmp/mapper.tum
export MOLA_MAPPER_OUTPUT_GEOREF=/tmp/mapper_georef.yaml
mola-cli $(ros2 pkg prefix mola_mapper)/share/mola_mapper/mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml
```

Every mapper setting is in `params/mapper.yaml` with an environment variable
(`MOLA_MAPPER_*`). `MOLA_WITH_MAPPER=false` runs LIO and the state estimator
only.

## Main outputs

Written at the end of the run, if the corresponding path is set:

| Parameter | Content |
|---|---|
| `output_tum_trajectory` | Optimized keyframe poses in the map frame (TUM format) |
| `output_simplemap` | `mrpt::maps::CSimpleMap` with the optimized keyframes and all their observations |
| `output_georeference` | Geo-reference (YAML: datum, `T_enu_to_map` and its sigmas) |
| `output_loop_closures` | Loop closure edges, one per line |
| `output_state_estimator_tum`, `output_state_estimator_georeference` | Trajectory and geo-reference of the state estimator, for comparisons |
| `output_graph_input` | Prefix of the files with the global graph input, for `mola-mapper-reoptimize` |
| `record_input` | Everything the mapper receives, in arrival order, for `mola-mapper-cli` |

The geo-reference is also published through `mola::MapSourceBase` when GNSS
factors are solved.

## mola-mapper-reoptimize

Offline tool that re-solves a global graph saved with `output_graph_input`,
using other graph parameters:

```bash
mola-mapper-reoptimize INPUT_PREFIX MAPPER_YAML OUTPUT_PREFIX [--no-loops] [--keep-initial]
```

It reads the `params.graph` section of `MAPPER_YAML` and writes
`OUTPUT_PREFIX.tum` and `OUTPUT_PREFIX_georef.yaml`. `--no-loops` ignores the
loop closures, and `--keep-initial` starts from the saved initial guesses
instead of chaining the front end increments from the first keyframe.

## mola-mapper-cli

Replays a mapper input recorded with `record_input` (`MOLA_MAPPER_RECORD_INPUT`) during
a run: keyframes, GNSS and IMU readings, and the gyroscope biases they got, in
arrival order. To tune the mapper (e.g. loop closure) without running the front end
again, and with the same result every time:

```bash
mola-mapper-cli INPUT_LOG MAPPER_YAML OUTPUT_DIR [--lc-period S] [--solve-period S] \
  [--save-simplemap] [--save-graph] [--threads N] [--quiet]
```

The mapper threads are disabled: loop closure scans and solves run at fixed periods
of data time (by default, those of `MAPPER_YAML`). With the default `--threads 1`,
two runs give the same output to the last bit; with more threads, parallel sums
make them differ slightly, and even the accepted loops may change. It writes
`mapper.tum`, `mapper_georef.yaml` and `loops.txt` into `OUTPUT_DIR`. The front end
does not run, so its reactions to the mapper (e.g. a LIO local map rebuilt after
loop closures) are those of the recorded run.

## Build and install

Refer to: https://docs.mola-slam.org/latest/#installing

## Documentation and tutorials

See: https://docs.mola-slam.org/latest/mola_mapper.html

## ROS build farm status

| Distro | Develop branch | Releases | Stable release |
| ---    | ---            | ---      |  ---      |
| ROS2 Humble  (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_mapper__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_mapper__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_mapper__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_mapper__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_mapper)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_mapper__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_mapper__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_mapper__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_mapper__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_mapper)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mola_mapper__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_mapper__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_mapper__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_mapper__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_mapper)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper) |
| ROS 2 Lyrical (u26.04) | [![Build Status](https://build.ros2.org/job/Ldev__mola_mapper__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mola_mapper__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Lbin_uR64__mola_mapper__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mola_mapper__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__mola_mapper__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__mola_mapper__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/lyrical/mola_mapper)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper) |
| ROS 2 Rolling (u26.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_mapper__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_mapper__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uR64__mola_mapper__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mola_mapper__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_mapper)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper) |


## Repository layout

```
module/include/mola_mapper/   Public headers (Mapper, GlobalGraph, GlobalGraphIO, GyroIntegration,
                              MapperInputLog, MapperReplay, Parameters)
module/src/                   Implementation
apps/                         mola-mapper-reoptimize, mola-mapper-cli
params/                       Mapper parameters and loop closure detector pipelines
mola-cli-launchs/             mola-cli launch files
docs/                         Sphinx/RST documentation pages
test/                         Unit tests
```

## Citation

The latest publication on MOLA is ([ArXiV](https://arxiv.org/abs/2407.20465)).

```bibtex
@article{blanco2025mola_lo,
    author = {Blanco-Claraco, Jose Luis},
    title ={{A flexible framework for accurate LiDAR odometry, map manipulation, and localization}},
    journal = {The International Journal of Robotics Research},
    volume = {44},
    number = {9},
    pages = {1553--1599},
    year = {2025},
    doi = {10.1177/02783649251316881},
    URL = { https://doi.org/10.1177/02783649251316881},
    eprint = {2407.20465},
}
```

## License
Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).

Contributions require acceptance of the Contributor License Agreement (CLA).
