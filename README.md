[![CI Build colcon](https://github.com/MOLAorg/mola_mapper_3d/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper_3d/actions/workflows/build-ros.yml)
[![CI clang-format](https://github.com/MOLAorg/mola_mapper_3d/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_mapper_3d/actions/workflows/check-clang-format.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/mola_mapper_3d.html)
[![codecov](https://codecov.io/gh/MOLAorg/mola_mapper_3d/graph/badge.svg?token=C11VFFK0NW)](https://codecov.io/gh/MOLAorg/mola_mapper_3d)

# mola_mapper_3d

Central 3D SLAM map for the [MOLA](https://github.com/MOLAorg/mola) framework,
compatible with ROS 2.

`mola_mapper_3d` holds **one global, optimized representation of the world**,
fusing multiple odometry sources (wheels, LiDAR-odometry, visual-odometry),
IMU and GNSS into a single keyframe-based map (a `mrpt::maps::CSimpleMap` backed
by a GTSAM iSAM2 factor graph), with anytime loop closure, geo-referencing,
lifelong keyframe management, and relocalization.

## Contents

This repository provides a C++ library `mola_mapper_3d` implementing the central
3D SLAM back-end.  It exposes the following `mola_kernel` interfaces:

| Interface | Role |
|---|---|
| `mola::NavStateFilter` | Single source of truth for short-term pose prediction queried by LIO/VIO front-ends |
| `mola::LocalizationSourceBase` | Fused vehicle localization (published at high rate) |
| `mola::MapSourceBase` | Map and geo-referencing |
| `mola::SharedKeyframeMap` | Receives keyframe-insert requests from LIO/VIO front-ends |
| `mola::DiagnosticsProvider` | Structured diagnostics |

ROS 2 example launch files are provided in [ros2-launchs](ros2-launchs/) and
[mola-cli-launchs](mola-cli-launchs/).

## Build and install

Refer to: https://docs.mola-slam.org/latest/#installing

## Documentation and tutorials

See: https://docs.mola-slam.org/latest/mola_mapper_3d.html

## ROS build farm status

| Distro | Develop branch | Releases | Stable release |
| ---    | ---            | ---      |  ---      |
| ROS2 Humble  (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_mapper_3d__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_mapper_3d__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_mapper_3d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_mapper_3d__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper_3d__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_mapper_3d__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_mapper_3d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_3d) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_mapper_3d__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_mapper_3d__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_mapper_3d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_mapper_3d__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper_3d__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_mapper_3d__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_mapper_3d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_3d) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mola_mapper_3d__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_mapper_3d__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_mapper_3d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_mapper_3d__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper_3d__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_mapper_3d__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_mapper_3d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_3d) |
| ROS 2 Lyrical (u26.04) | [![Build Status](https://build.ros2.org/job/Ldev__mola_mapper_3d__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mola_mapper_3d__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Lbin_uR64__mola_mapper_3d__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mola_mapper_3d__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__mola_mapper_3d__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__mola_mapper_3d__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/lyrical/mola_mapper_3d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_3d) |
| ROS 2 Rolling (u26.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_mapper_3d__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_mapper_3d__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uR64__mola_mapper_3d__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mola_mapper_3d__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper_3d__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_mapper_3d__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_mapper_3d)](https://index.ros.org/?search_packages=true&pkgs=mola_mapper_3d) |


## Repository layout

```
module/include/mola_mapper_3d/   Public headers (Mapper3D, Parameters, WorldModelState)
module/src/                      Implementation (split by concern)
apps/                            mola-mapper-3d-cli (offline front-end)
params/                          Default YAML configuration
mola-cli-launchs/                mola-cli launch files
ros2-launchs/                    ROS 2 launch files
docs/                            Sphinx/RST documentation pages
test/                            Unit tests
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
