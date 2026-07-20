.. _mola_mapper:

================================
3D central SLAM map (Mapper)
================================

:octicon:`mark-github` `mola_mapper <https://github.com/MOLAorg/mola_mapper/>`_ is the
**central 3D SLAM back-end** for the MOLA framework.
It holds **one global, optimized representation of the world** by fusing multiple
odometry sources (wheel encoders, LiDAR-odometry, visual-odometry), IMU and GNSS
into a single keyframe-based factor graph (GTSAM iSAM2), with online loop closure,
geo-referencing, and lifelong keyframe management.

Mapper acts as the **single source of truth** for pose prediction queried by any
number of LIO/VIO front-ends running concurrently, replacing the lighter-weight
:ref:`state estimators <mola_sta_est_index>` in full SLAM use cases.

.. note::
  This module is under active development. The core fusion pipeline (multi-source
  odometry, IMU, GNSS, keyframe-ingestion from LIO/VIO) is operational.
  Loop closure, map load/save, and relocalization are planned for upcoming releases.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

.. _mola_mapper_role:

Role within the MOLA ecosystem
----------------------------------

At a conceptual level, Mapper is the **global back-end** that:

1. Receives **short-term pose estimates** from one or more front-ends
   (e.g. :ref:`mola_lidar_odometry <mola_lidar_odometry>`) via the
   ``mola::SharedKeyframeMap`` and ``mola::NavStateFilter::fuse_pose()`` interfaces.
2. Fuses IMU attitude/gravity/gyro factors and GNSS ENU factors into the same graph.
3. Runs **incremental iSAM2 optimization** to refine all keyframe poses globally,
   including past ones affected by loop closures.
4. Publishes the **fused localization** back to front-ends and to ROS 2.

The relationship between front-ends and Mapper is symmetric: the front-end drives
short-term motion estimation, while Mapper provides the globally-consistent anchor
and corrects for drift over time.

.. figure:: imgs/mapper_system_scheme.png
   :width: 640
   :alt: Mapper system block diagram

   Mapper receives pose streams from LIO/VIO front-ends and fuses them with IMU
   and GNSS into one optimized world model.

|

.. _mola_mapper_interfaces:

Provided interfaces
-----------------------

Mapper implements the following ``mola_kernel`` interfaces:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Interface
     - Role
   * - ``mola::NavStateFilter``
     - Provides ``estimated_navstate()`` (pose + twist + covariance) for any
       requested timestamp, using the latest iSAM2 solution plus a
       constant-velocity kinematic extrapolator.
   * - ``mola::LocalizationSourceBase``
     - Publishes the fused vehicle localization (``advertiseUpdatedLocalization``)
       at the configured high-rate.
   * - ``mola::MapSourceBase``
     - Publishes the map and geo-referencing transforms.
   * - ``mola::SharedKeyframeMap``
     - Accepts keyframe-insert requests from LIO/VIO front-ends via
       ``requestInsertKeyframe()``, merging raw observations and adding
       tight consecutive-frame relative-pose factors.
   * - ``mola::DiagnosticsProvider``
     - Exposes structured diagnostics (keyframe count, convergence status,
       GNSS/IMU factor count, odom drift per source).

|

.. _mola_mapper_frames:

Reference frame conventions
-------------------------------

Mapper manages three categories of frames:

* **{map}** (configurable via ``reference_frame_name``, default ``map``): the
  global optimization frame.  All keyframe poses are expressed in this frame.
* **{odom_i}** (one per odometry source, e.g. ``odom``, ``odom_lidar``): the
  short-term prediction frame served to each front-end. Mapper estimates
  ``T_map_to_odom_i`` for each source and corrects it after each iSAM2 solve.
* **{enu}** (``enu_frame_name``, default ``enu``): the ENU geo-reference frame,
  estimated or fixed.  ``T_enu_to_map`` is a variable in the factor graph when
  geo-referencing is enabled.

This mirrors the conventions of :ref:`mola_state_estimators <mola_sta_est_index>`.

|

.. _mola_mapper_params:

Configuration parameters
----------------------------

All parameters are loaded from the ``params:`` subtree of the module YAML.
The shared defaults live in ``params/mapper-common.yaml`` and are imported
via the ``$import`` directive in per-dataset launch files.

Below is a summary of the most frequently tuned parameters, grouped by topic.
All of them can be overridden via environment variables (listed in parentheses).

.. dropdown:: Reference frames
   :icon: list-unordered

   .. code-block:: yaml

      vehicle_frame_name: "${MOLA_TF_BASE_LINK|base_link}"
      reference_frame_name: "${MOLA_TF_MAP|map}"
      enu_frame_name: "enu"

.. dropdown:: Kinematic model and keyframe creation
   :icon: list-unordered

   .. code-block:: yaml

      # Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle
      kinematic_model: "${MOLA_MAPPER_KINEMATIC_MODEL|KinematicModel::ConstantVelocity}"

      max_time_to_use_velocity_model: 0.75   # [s]  (MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL)
      min_time_difference_to_create_new_frame: 0.05  # [s]

      # Bound high-rate sensor insertion rate (0 = insert every reading):
      imu_max_insert_rate_hz: 5.0    # (MOLA_MAPPER_IMU_MAX_RATE_HZ)
      odometry_max_insert_rate_hz: 5.0  # (MOLA_MAPPER_ODOM_MAX_RATE_HZ)

      # Share a bounded-rate keyframe clock for IMU + wheels:
      aggregate_high_rate_into_edges: false  # (MOLA_MAPPER_AGGREGATE)
      sensor_keyframe_min_period: 0.5        # [s] (MOLA_MAPPER_SENSOR_KF_PERIOD)

      enforce_planar_motion: false   # (MOLA_MAPPER_ENFORCE_PLANAR_MOTION)

.. dropdown:: IMU fusion
   :icon: list-unordered

   .. code-block:: yaml

      imu_attitude_sigma_deg: 2.0        # (MOLA_IMU_ATTITUDE_SIGMA)
      imu_attitude_azimuth_offset_deg: 0.0  # (MOLA_IMU_ATTITUDE_AZIMUTH_OFFSET)
      # Sigma for gravity-leveling from accelerometer (0 = disable):
      imu_normalized_gravity_alignment_sigma: 0.4  # (MOLA_IMU_GRAVITY_ALIGNMENT_SIGMA)
      # Per-keyframe gyro prior sigma [deg/s] (0 = disable):
      imu_angular_velocity_sigma_deg: 0.0  # (MOLA_IMU_ANGVEL_SIGMA)

.. dropdown:: Geo-referencing
   :icon: list-unordered

   .. code-block:: yaml

      estimate_geo_reference: false   # (MOLA_ESTIMATE_GEO_REF)
      gnss_huber_threshold: 1.5       # [sigmas] (MOLA_GNSS_HUBER_THRESHOLD)
      convergence_max_position_sigma: 1.0   # [m] (MOLA_MAPPER_CONV_POS_SIGMA)
      convergence_max_orientation_sigma_deg: 5.0
      publish_estimated_georef_on_convergence: true

.. dropdown:: Nonlinear optimization
   :icon: list-unordered

   .. code-block:: yaml

      additional_isam2_update_steps: 3
      # Background thread for iSAM2 (recommended for online use):
      enable_optimizer_thread: true   # (MOLA_MAPPER_OPTIMIZER_THREAD)

.. dropdown:: High-rate pose publisher
   :icon: list-unordered

   .. code-block:: yaml

      high_rate_pose_publish_rate_hz: 0.0  # [Hz] 0 = disabled (MOLA_MAPPER_PUBLISH_RATE_HZ)
      high_rate_use_latest_sensors: true

.. dropdown:: Map save / load
   :icon: list-unordered

   .. code-block:: yaml

      load_simplemap_file: ""   # path to .simplemap (MOLA_MAPPER_LOAD_SM)
      save_simplemap_file: ""   # path to .simplemap (MOLA_MAPPER_SAVE_SM)
      externalize_after_seconds: 30.0

.. dropdown:: Output trajectory (TUM format)
   :icon: list-unordered

   .. code-block:: yaml

      # If non-empty, save estimated trajectory at shutdown (and on demand via
      # the GUI "Output" tab).  Reference frame = reference_frame_name ({map}).
      save_trajectory_to_file: ""   # (MOLA_MAPPER_TUM_TRAJECTORY_OUTPUT)

   The file is written in
   `TUM format <https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format>`_
   (``timestamp tx ty tz qx qy qz qw``), compatible with
   `evo <https://github.com/MichaelGrupp/evo>`_ for trajectory evaluation.

   Because past poses may be revised by the global optimizer after new data
   arrives, the file written **at shutdown** is the most accurate snapshot.
   The "Save trajectory now" button in the GUI captures the current best estimate
   at any point during a run.

.. dropdown:: Sensor label filters (regex)
   :icon: list-unordered

   .. code-block:: yaml

      do_process_imu_labels_re: ".*"       # (MOLA_MAPPER_IMU_LABELS_RE)
      do_process_odometry_labels_re: ".*"  # (MOLA_MAPPER_ODOMETRY_LABELS_RE)
      do_process_gnss_labels_re: ".*"      # (MOLA_MAPPER_GNSS_LABELS_RE)

|

.. _mola_mapper_gui:

GUI sub-window
-----------------

When a ``mola_viz`` or ``mola_viz_imgui`` visualizer module is loaded, Mapper
creates a floating sub-window with four tabs:

* **Status** — live counts of keyframes, graph edges, odometry frames, IMU
  factors, and localization convergence status.
* **Geo-ref** — GNSS factor count, tentative ENU origin, current ``T_enu_to_map``
  estimate and sigmas, and per-source odometry-drift report.
* **View** — toggles for keyframe corners, graph edges, odometry-frame axes, and
  camera-follows-vehicle.
* **Output** — filename edit box and "Save trajectory now" button for on-demand
  TUM trajectory export.

|

.. _mola_mapper_ros2:

ROS 2 integration
-----------------------

ROS 2 launch files are provided in the ``ros2-launchs/`` directory.
See :ref:`mola_mapper_ros2_node` for a description of published topics,
subscribed topics, parameters, and ``/tf`` conventions.

|
