/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   Mapper3D_SensorCallbacks.cpp
 * @brief  Dispatch of incoming raw observations to the proper fuse_*() method.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>

namespace mola::mapper
{

void Mapper::onNewObservation(const CObservation::ConstPtr & o)
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "onNewObservation");
  if (!o) {
    return;
  }

  // Wheel odometry:
  if (auto odo = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o); odo) {
    if (std::regex_match(odo->sensorLabel, odometry_labels_re_)) {
      fuse_odometry(*odo, odo->sensorLabel.empty() ? "odom_wheels" : odo->sensorLabel);
    }
    return;
  }

  // IMU:
  if (auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o); imu) {
    if (std::regex_match(imu->sensorLabel, imu_labels_re_)) {
      fuse_imu(*imu);
    }
    return;
  }

  // GNSS:
  if (auto gps = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o); gps) {
    if (std::regex_match(gps->sensorLabel, gnss_labels_re_)) {
      fuse_gnss(*gps);
    }
    return;
  }

  // Other observation classes are ignored by Mapper (LiDAR/visual front ends
  // feed it via fuse_pose() and, in later phases, the keyframe-insertion API).
  MRPT_END
}

}  // namespace mola::mapper
