.. _mola_mapper_ros2_node:

==============================
Mapper ROS 2 node
==============================

.. note::
  ROS 2 launch files and a dedicated bridge node for ``mola_mapper`` are
  under active development.  This page will be populated with topic names,
  parameters, and ``/tf`` conventions once the bridge is finalised.
  In the meantime, refer to the mola-cli launch files in
  ``mola-cli-launchs/`` for working examples that combine LIO + Mapper.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

Overview
-----------

The Mapper ROS 2 integration consists of:

1. A ``mola_bridge_ros2`` adapter that subscribes to sensor topics
   (``nav_msgs/Odometry``, ``sensor_msgs/Imu``, ``sensor_msgs/NavSatFix``)
   and forwards them to the Mapper module via the MOLA sensor-fusion API.
2. A publisher for ``nav_msgs/Odometry`` and ``/tf`` (``map → base_link``),
   driven by ``mola::LocalizationSourceBase::advertiseUpdatedLocalization()``.
3. (Planned) A ``nav_msgs/Path`` publisher for the full optimized trajectory.

|

.. _mola_mapper_ros2_launch:

Launch file
-----------

*Coming soon.*  The launch file will accept the following key arguments:

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Argument
     - Default
     - Description
   * - ``lidar_topic_name``
     - ``/ouster/points``
     - Raw LiDAR topic consumed by the paired LIO front-end.
   * - ``imu_topic_name``
     - ``""``
     - IMU topic (``sensor_msgs/Imu``); empty = disabled.
   * - ``gnss_topic_name``
     - ``""``
     - GNSS topic (``sensor_msgs/NavSatFix``); empty = disabled.
   * - ``estimate_geo_reference``
     - ``false``
     - Enable online geo-referencing from GNSS.
   * - ``odom_topic``
     - ``/odom``
     - Wheel-odometry topic (``nav_msgs/Odometry``).
   * - ``base_link_frame``
     - ``base_link``
     - Robot base frame id.
   * - ``map_frame``
     - ``map``
     - Global map frame id.

|

.. _mola_mapper_ros2_topics:

Published topics
-----------------

*Coming soon.*

Subscribed topics
------------------

*Coming soon.*

TF conventions
---------------

*Coming soon.*
