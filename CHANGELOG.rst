^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_mapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-11-20)
------------------
* First release: mola_mapper as a global back end next to LIO and the state estimator: keyframe sink and batch global pose graph (odometry chain, loop closures, GNSS, gravity, gyroscope and wheel odometry factors).
* Loop closure detection with mola_sm_loop_closure (BEV global registration by default), odometry and pairwise consistency checks.
* Acts as a mola::LocalMapCorrectionSource, so front ends get corrected keyframe poses after loop closures.
* Attach up to three camera images to keyframes.
* GUI: keyframe graph in the visualizer, map from keyframe clouds, animated map corrections and scripted camera.
* mola-mapper-cli (offline mapping, record and bit-exact replay of the mapper input) and mola-mapper-reoptimize.
* Launchers: rosbag2 / ROS 1 bags, live ROS 2 topics, and datasets profiled in mola_lidar_odometry.
* Builds with MRPT 3.x and with Boost or non-Boost GTSAM.
* Contributors: Jose Luis Blanco-Claraco
