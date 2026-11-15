.. _mola_mapper_ros2_node:

==============================
Running Mapper with a rosbag2
==============================

The mapper has no ROS 2 node of its own: it is a MOLA module, loaded together
with the other modules of the system by ``mola-cli``. The provided launch file
reads a rosbag2 and runs LIO, the state estimator and the mapper.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

.. _mola_mapper_ros2_launch:

Launch file
-----------

``mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml`` (installed in
``share/mola_mapper/mola-cli-launchs``) loads:

* ``mola::Rosbag2Dataset``: reads the bag and feeds the other modules.
* ``mola::LidarOdometry``: the front end (pipeline from ``mola_lidar_odometry``).
* ``mola::state_estimation_smoother::StateEstimationSmoother``: the state
  estimator queried by LIO.
* ``mola::mapper::Mapper``: parameters from ``params/mapper.yaml``.
* Optionally, a GUI module.

.. code-block:: bash

   export MOLA_INPUT_ROSBAG2=/path/to/bag
   export MOLA_LIDAR_TOPIC=/ouster/points
   export MOLA_IMU_TOPIC=/ouster/imu
   export MOLA_GNSS_TOPIC=/gps/fix          # optional
   export MOLA_MAPPER_OUTPUT_TUM=/tmp/mapper.tum
   mola-cli $(ros2 pkg prefix mola_mapper)/share/mola_mapper/mola-cli-launchs/lio_smoother_mapper_from_rosbag2.yaml

Environment variables of the launch file:

.. list-table::
   :widths: 35 20 45
   :header-rows: 1

   * - Variable
     - Default
     - Description
   * - ``MOLA_INPUT_ROSBAG2``
     - (required)
     - Path to the rosbag2.
   * - ``MOLA_LIDAR_TOPIC``
     - ``/ouster/points``
     - LiDAR topic (sensor label ``lidar``).
   * - ``MOLA_IMU_TOPIC``
     - ``/imu``
     - IMU topic (sensor label ``imu``), optional.
   * - ``MOLA_ATTITUDE_IMU_TOPIC``, ``MOLA_ATTITUDE_IMU_NAME``
     - empty, ``imu_attitude``
     - Optional second IMU topic and its sensor label.
   * - ``MOLA_GNSS_TOPIC``
     - empty
     - GNSS topic (sensor label ``gps``), optional.
   * - ``MOLA_CAMERA_TOPIC``, ``_1``, ``_2``
     - empty
     - Camera image topics (labels ``camera``, ``camera_1``, ``camera_2``), optional. Stored in
       the mapper keyframes with ``MOLA_MAPPER_IMAGE_LABELS='camera.*'``.
   * - ``MOLA_ODOMETRY_TOPIC``
     - empty
     - Wheel odometry topic (sensor label ``odom_wheels``), optional.
   * - ``MOLA_TF_BASE_LINK``, ``MOLA_TF_TOPIC``, ``MOLA_TF_STATIC_TOPIC``
     - ``base_link``, ``/tf``, ``/tf_static``
     - Vehicle frame and TF topics read from the bag.
   * - ``MOLA_TF_MAP``
     - ``map``
     - Map frame (``reference_frame_name`` of the mapper).
   * - ``MOLA_WITH_MAPPER``
     - ``true``
     - ``false`` runs LIO and the state estimator only.
   * - ``MOLA_WITH_GUI``
     - ``true``
     - Enable the GUI and the LiDAR preview.
   * - ``MOLA_TIME_WARP``, ``MOLA_DATASET_START_PAUSED``, ``MOLA_QUIT_ON_DATASET_END``
     - ``1.0``, ``false``, ``false``
     - Dataset playback controls.
   * - ``MOLA_ODOMETRY_PIPELINE_YAML``
     - ``lidar3d-gicp.yaml``
     - LIO pipeline file.
   * - ``MOLA_STATE_ESTIMATOR_YAML``
     - smoother default
     - State estimator parameters.
   * - ``MOLA_MAPPER_YAML``
     - ``../params/mapper.yaml``
     - Mapper parameters (relative to the launch file).

The mapper settings (``MOLA_MAPPER_*``) are listed in :ref:`mola_mapper_params`.
Useful combinations:

* A live geo-referenced estimate too: ``MOLA_ESTIMATE_GEO_REF=true`` (the state
  estimator geo-references itself on the fly from the same GNSS readings as the
  mapper).
* Gyroscope factors: ``MOLA_MAPPER_GYRO_LABELS=imu``, optionally with
  ``MOLA_NAVSTATE_IMU_BIAS_TRACKING=true`` in the state estimator.
* Gravity factors: ``MOLA_MAPPER_GRAVITY_IMU_LABELS=imu``.

When the bag ends, stop the application (or set ``MOLA_QUIT_ON_DATASET_END=true``):
the mapper then runs its final loop closure passes and writes the outputs.

|

Topics and TF
--------------

The launch file loads no ROS 2 bridge module, so it publishes no ROS 2 topics
nor ``/tf``. Results are read from the output files (see :ref:`mola_mapper_params`).
