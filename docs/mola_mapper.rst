.. _mola_mapper:

================================
Global back end (Mapper)
================================

:octicon:`mark-github` `mola_mapper <https://github.com/MOLAorg/mola_mapper/>`__ is the
**global back end** of a MOLA system. It runs next to a front end
(:ref:`LiDAR odometry <mola_lidar_odometry>`) and its state estimator, and
adds a central keyframe map, loop closure detection and a global pose graph
that includes GNSS, gravity and gyroscope factors. It does little estimation of
its own: the state estimator remains the only ``mola::NavStateFilter`` queried by
the front end.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

.. _mola_mapper_role:

Role within the MOLA ecosystem
----------------------------------

The modules of a system are independent and composed by a launch file:

* ``mola::LidarOdometry`` (LIO) uses the state estimator
  (``mola::state_estimation_smoother::StateEstimationSmoother``) as its only
  ``NavStateFilter``, and pushes sparse keyframes to the mapper through
  ``mola::SharedKeyframeMap``.
* ``mola::mapper::Mapper`` receives those keyframes and, as a
  ``mola::RawDataConsumer``, the GNSS and IMU readings of the dataset.
* Optionally (``feedback_to_state_estimator``), after each solve the mapper
  fuses the optimized pose of the newest keyframe (once per keyframe) into the
  state estimator as an absolute pose in the map frame (``fuse_pose()``), so loop
  closures and GNSS reach the live estimate. In that mode, the state estimator should run without
  GNSS (``MOLA_NAVSTATE_GNSS_SENSOR_NAME=none``): the mapper owns it.

The mapper:

1. Attaches to each keyframe the nearest GNSS reading, the accelerometer
   averaged before it (gravity) and the gyroscope rotation since the previous
   keyframe.
2. Searches for loop closures in a background thread, with the detector of
   ``mola_sm_loop_closure``.
3. Optimizes the pose graph of all keyframes in another background thread.
4. Publishes the estimated geo-reference and, at the end of the run, writes the
   optimized trajectory, map and geo-reference.

|

.. _mola_mapper_interfaces:

Provided interfaces
-----------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Interface
     - Role
   * - ``mola::SharedKeyframeMap``
     - ``requestInsertKeyframe()``: receives the keyframes of the front end
       (source frame, pose and covariance in that frame, observations).
   * - ``mola::RawDataConsumer``
     - Receives GNSS and IMU readings from the ``raw_data_source`` modules.
   * - ``mola::MapSourceBase``
     - Publishes the estimated geo-reference.
   * - ``mola::DiagnosticsProvider``
     - Keyframe, loop closure and factor counts, solves and loop closure scans.
   * - ``mola::ExecutableBase``
     - Module lifecycle. ``onQuit()`` runs the end-of-run passes and writes the
       outputs.

|

.. _mola_mapper_frames:

Reference frame conventions
-------------------------------

* **{map}** (``reference_frame_name``, default ``map``): the frame of the
  optimized keyframe poses. It is the one of the front end keyframes: the first
  keyframe stays at its front end pose, which is also where the state estimator
  starts.
* **{enu}**: the geo-reference ``T_enu_to_map`` is a variable of the graph when
  there are GNSS readings. It is published as a ``Georeferencing`` and can be
  saved to a YAML file.

|

.. _mola_mapper_graph:

Global pose graph
----------------------

The graph has one pose variable per keyframe in ``{map}`` and ``T_enu_to_map``.
It is solved in batch with Levenberg-Marquardt (GTSAM), and the marginal
covariance of the newest pose is computed. Its factors are:

* **Odometry chain**: between consecutive keyframes of the same front end source
  frame. Stiff by default. The sigma is a floor plus terms per square root of
  meter and per radian turned (``odometry_noise_ang_per_rad``). Keyframes of
  different source frames are joined by a weak edge.
* **Loop closures**: Huber kernel, with floors on their sigmas
  (``loop_closure_min_sigma_xyz``, ``loop_closure_min_sigma_ang_deg``), since the
  detector covariance is overconfident.
* **GNSS**: ENU position factors (readings with a GGA message) with a selectable
  robust kernel (default Cauchy). Readings with a horizontal sigma above
  ``gnss_max_uncertainty_horiz`` are discarded. If the spread of the readings is
  too small for their sigmas, they are not used. The geo-reference is published
  and saved only once observable: at least ``georef_min_gnss_factors`` GNSS
  factors, and an azimuth sigma of ``T_enu_to_map`` below
  ``georef_max_sigma_yaw_deg``.
* **Gravity**: roll and pitch from the averaged accelerometer.
* **Gyroscope**: relative rotation between consecutive keyframes, with free
  translation. The sigma combines the noise density, the bias sigma and the scale
  error.
* A tight prior on the first keyframe (gauge) and a weak prior on
  ``T_enu_to_map``.

|

.. _mola_mapper_params:

Configuration parameters
----------------------------

All parameters are loaded from the ``params:`` subtree of the module YAML. The
defaults are in ``params/mapper.yaml``, where most of them can be overridden
with an environment variable (shown in parentheses below).

.. code-block:: yaml

   - name: mapper
     type: mola::mapper::Mapper
     raw_data_source: "dataset_input"   # GNSS and IMU readings
     params: "../params/mapper.yaml"

.. dropdown:: Frame and sensors attached to the keyframes
   :icon: list-unordered

   .. code-block:: yaml

      reference_frame_name: "${MOLA_TF_MAP|map}"

      # GNSS (regex of sensor labels; empty: none)
      gnss_sensor_labels_re: "${MOLA_MAPPER_GNSS_LABELS|gps}"
      gnss_max_time_offset: 0.5          # [s] (MOLA_MAPPER_GNSS_MAX_TIME_OFFSET)

      # IMU whose averaged accelerometer levels the map (empty: none)
      imu_sensor_labels_re: "${MOLA_MAPPER_GRAVITY_IMU_LABELS|}"
      imu_gravity_window: 1.0            # [s] before each keyframe (MOLA_MAPPER_GRAVITY_WINDOW)
      imu_gravity_min_samples: 10

      # IMU whose gyroscope gives the rotation between keyframes (empty: none)
      gyro_sensor_labels_re: "${MOLA_MAPPER_GYRO_LABELS|}"
      gyro_noise_density: 3e-4           # [rad/s/sqrt(Hz)] (MOLA_MAPPER_GYRO_NOISE_DENSITY)
      gyro_bias_sigma: 0.005             # [rad/s], without a bias source estimate
      gyro_scale_sigma: 0.005            # (MOLA_MAPPER_GYRO_SCALE_SIGMA)
      gyro_max_gap: 0.1                  # [s] longer gaps between readings: no factor

   The gyroscope bias comes from the first ``mola::ImuBiasSource`` module with an
   estimate (e.g. the state estimator with ``MOLA_NAVSTATE_IMU_BIAS_TRACKING=true``).
   Otherwise it is taken as zero, with sigma ``gyro_bias_sigma``.

.. dropdown:: Graph: odometry chain
   :icon: list-unordered

   .. code-block:: yaml

      graph:
        odometry_uncertainty_multiplier: 0.0       # front end covariance, 0: ignored (MOLA_MAPPER_ODOM_SIGMA_MULTIPLIER)
        odometry_additional_noise_xyz: 0.002       # [m] (MOLA_MAPPER_ODOM_SIGMA_XYZ)
        odometry_additional_noise_ang_deg: 0.01    # [deg] (MOLA_MAPPER_ODOM_SIGMA_ANG)
        odometry_noise_xyz_per_sqrt_m: 0.01        # (MOLA_MAPPER_ODOM_SIGMA_XYZ_PER_SQRT_M)
        odometry_noise_ang_deg_per_sqrt_m: 0.05    # (MOLA_MAPPER_ODOM_SIGMA_ANG_PER_SQRT_M)
        odometry_noise_ang_per_rad: 0.02           # (MOLA_MAPPER_ODOM_SIGMA_ANG_PER_RAD)

.. dropdown:: Graph: gyroscope, GNSS, gravity and loop closure factors
   :icon: list-unordered

   .. code-block:: yaml

      graph:
        use_gyro: true                  # (MOLA_MAPPER_USE_GYRO)
        gyro_min_sigma_deg: 0.02

        use_gnss: true                  # (MOLA_MAPPER_USE_GNSS)
        gnss_minimum_uncertainty: 0.5   # [m] (MOLA_MAPPER_GNSS_MIN_SIGMA)
        gnss_uncertainty_multiplier: 1.0   # (MOLA_MAPPER_GNSS_SIGMA_MULTIPLIER)
        gnss_max_uncertainty_horiz: 20.0   # [m]
        gnss_robust_kernel: "cauchy"    # huber, cauchy, geman_mcclure, tukey (MOLA_MAPPER_GNSS_ROBUST_KERNEL)
        gnss_robust_k: 1.5              # [sigmas] (MOLA_MAPPER_GNSS_ROBUST_K)
        georef_min_gnss_factors: 3
        georef_max_sigma_yaw_deg: 10.0  # [deg] (MOLA_MAPPER_GEOREF_MAX_SIGMA_YAW)

        use_imu_gravity: true           # (MOLA_MAPPER_USE_GRAVITY)
        imu_gravity_sigma_deg: 3.0      # (MOLA_MAPPER_GRAVITY_SIGMA_DEG)

        loop_closure_robust_huber_k: 1.5
        loop_closure_min_sigma_xyz: 0.05      # [m] (MOLA_MAPPER_LC_MIN_SIGMA_XYZ)
        loop_closure_min_sigma_ang_deg: 0.25  # [deg] (MOLA_MAPPER_LC_MIN_SIGMA_ANG)

.. dropdown:: Optimizer and feedback to the state estimator
   :icon: list-unordered

   .. code-block:: yaml

      enable_optimizer_thread: true
      optimizer_min_period: 1.0          # [s] (MOLA_MAPPER_OPTIMIZER_PERIOD)

      feedback_to_state_estimator: false # (MOLA_MAPPER_FEEDBACK)
      feedback_min_sigma_xyz: 0.10       # [m] (MOLA_MAPPER_FEEDBACK_MIN_SIGMA_XYZ)
      feedback_min_sigma_ang_deg: 0.5    # [deg] (MOLA_MAPPER_FEEDBACK_MIN_SIGMA_ANG)

   With ``feedback_to_state_estimator``, there must be exactly one
   ``mola::NavStateFilter`` module in the system.

.. dropdown:: Loop closure
   :icon: list-unordered

   .. code-block:: yaml

      loop_closure_enabled: true         # (MOLA_MAPPER_LOOP_CLOSURE)
      loop_closure_pipeline_file: "$(ros2 pkg prefix mola_mapper)/share/mola_mapper/params/loop-closure-f2f-mapper.yaml"  # (MOLA_MAPPER_LC_PIPELINE)
      loop_closure_check_period_sec: 5.0 # [s] (MOLA_MAPPER_LC_PERIOD)
      loop_closure_min_new_keyframes: 10
      loop_closure_incremental: true
      loop_closure_full_scan_every_n: 10
      loop_closure_finalize_rounds: 4    # (MOLA_MAPPER_LC_FINALIZE_ROUNDS)

   The detector is the ``mola_sm_loop_closure`` frame-to-frame engine, used only
   to find candidate edges. Scans are incremental (pairs with a new keyframe),
   with a full one every ``loop_closure_full_scan_every_n``. At the end of the
   run, up to ``loop_closure_finalize_rounds`` passes of full scan plus
   re-optimization run until no new loop appears. Note that the default of
   ``loop_closure_enabled`` in the C++ ``Parameters`` struct is ``false``; the
   provided YAML enables it.

.. dropdown:: Outputs
   :icon: list-unordered

   .. code-block:: yaml

      # Written at the end of the run (empty: not written)
      output_tum_trajectory: ""                 # (MOLA_MAPPER_OUTPUT_TUM)
      output_simplemap: ""                      # (MOLA_MAPPER_OUTPUT_SIMPLEMAP)
      output_georeference: ""                   # (MOLA_MAPPER_OUTPUT_GEOREF)
      output_loop_closures: ""                  # (MOLA_MAPPER_OUTPUT_LOOPS)
      output_state_estimator_tum: ""            # (MOLA_MAPPER_OUTPUT_SE_TUM)
      output_state_estimator_georeference: ""   # (MOLA_MAPPER_OUTPUT_SE_GEOREF)
      output_graph_input: ""                    # (MOLA_MAPPER_OUTPUT_GRAPH_INPUT)

   * ``output_tum_trajectory``: optimized keyframe poses in ``{map}``, in
     `TUM format <https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format>`_
     (``timestamp tx ty tz qx qy qz qw``), compatible with
     `evo <https://github.com/MichaelGrupp/evo>`_.
   * ``output_simplemap``: ``mrpt::maps::CSimpleMap`` with the optimized
     keyframes and all their observations.
   * ``output_georeference``: YAML with the datum, ``T_enu_to_map`` and its sigmas.
   * ``output_loop_closures``: one edge per line.
   * ``output_state_estimator_*``: the trajectory and geo-reference of the state
     estimator, for comparisons (the smoother needs
     ``keep_finalized_trajectory`` to return its trajectory).
   * ``output_graph_input``: file prefix of the graph input, to re-solve it
     offline with :ref:`mola-mapper-reoptimize <mola_mapper_reoptimize>`.

|

.. _mola_mapper_reoptimize:

mola-mapper-reoptimize
---------------------------

Offline tool that re-solves a global graph saved with ``output_graph_input``,
to try other graph parameters without running the whole system again:

.. code-block:: bash

   mola-mapper-reoptimize INPUT_PREFIX MAPPER_YAML OUTPUT_PREFIX [--no-loops] [--keep-initial]

* ``INPUT_PREFIX``: the ``output_graph_input`` prefix.
* ``MAPPER_YAML``: a mapper parameters file; only its ``params.graph`` section is
  used (environment variables apply, as in a launch).
* ``OUTPUT_PREFIX``: writes ``OUTPUT_PREFIX.tum`` and
  ``OUTPUT_PREFIX_georef.yaml``.
* ``--no-loops``: ignore the loop closures.
* ``--keep-initial``: start from the saved initial guesses. By default, the front
  end increments are chained from the first keyframe.

|

.. _mola_mapper_ros2:

ROS 2 integration
-----------------------

See :ref:`mola_mapper_ros2_node` for how to run the provided launch file with a
rosbag2.

|
