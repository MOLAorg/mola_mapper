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
 * @file   CameraDirector.h
 * @brief  Scripted GUI camera: map overviews and close-ups of the vehicle.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>

#include <optional>
#include <string>
#include <utility>

namespace mola::mapper
{
/// Scripted GUI camera (e.g. for videos). Times in wall clock seconds.
struct CameraDirectorParams
{
  bool enabled = false;
  /// Camera updates per second.
  double rate = 30.0;

  /// Overview: the whole map (keyframes plus a margin) seen from above.
  double overview_elevation = 90.0;  //!< [deg]
  /// Fixed azimuth of the overview [deg] (-90: +y up on the screen).
  double overview_azimuth = -90.0;
  double overview_margin = 15.0;        //!< [m] around the keyframes
  double overview_min_distance = 60.0;  //!< [m]

  /// Close-up: behind the vehicle.
  double closeup_elevation = 50.0;  //!< [deg]
  double closeup_distance = 40.0;   //!< [m]

  /// Schedule: overview, transition, close-up, transition, again.
  double overview_period = 40.0;
  double closeup_period = 15.0;
  double transition = 4.0;

  /// Time constants [s] of the low-pass filters of the vehicle heading, the
  /// vehicle position and the map extent.
  double heading_time_constant = 3.0;
  double vehicle_time_constant = 0.5;
  double map_time_constant = 2.0;

  /// Camera vertical field of view [deg] and image aspect ratio, to fit the map.
  double fov = 30.0;
  double aspect_ratio = 16.0 / 9.0;

  /// Overview only during the last seconds of the data (if the data source
  /// tells its time, e.g. a dataset player). Not used by CameraDirector.
  double final_overview = 60.0;

  /** Correction shots: when the map is corrected (e.g. a loop closure) by
   *  more than `shot_min_correction` [m] (0: never), the camera flies to the
   *  corrected region (`transition` s), shows it for `shot_hold` s (the map
   *  glides into place meanwhile), then flies back to the schedule. */
  double shot_min_correction = 0.5;
  double shot_elevation = 60.0;      //!< [deg]
  double shot_hold = 6.0;            //!< [s]
  double shot_margin = 20.0;         //!< [m] around the corrected region
  double shot_min_distance = 40.0;   //!< [m]
  double shot_max_distance = 250.0;  //!< [m]
  double shot_min_interval = 20.0;   //!< [s] between shot starts

  /// If not empty, the camera blend weights are written here over time (CSV),
  /// e.g. to speed up a video differently in each part.
  std::string timeline_file;

  void loadFrom(const mrpt::containers::yaml & cfg);
};

/** Camera look-at, azimuth, elevation and distance over time: a smooth blend
 *  of two views, a map overview (top view fitted to the keyframes) and a
 *  close-up behind the vehicle, alternated by a fixed schedule. Once
 *  Inputs::final is set (e.g. near the end of the data), it stays in the
 *  overview. All inputs go through low-pass filters, so the camera never
 *  jumps (e.g. on loop closure corrections or angle wrap-arounds). */
class CameraDirector
{
public:
  explicit CameraDirector(const CameraDirectorParams & p) : params_(p) {}

  struct Inputs
  {
    std::optional<mrpt::math::TPoint3D> vehicle;  //!< in the map frame
    std::optional<double> vehicle_yaw;            //!< [rad]
    std::optional<mrpt::math::TBoundingBox> map;  //!< keyframes
    bool final = false;                           //!< overview from now on
  };

  struct Camera
  {
    mrpt::math::TPoint3D look_at;
    double azimuth = 0;    //!< [rad]
    double elevation = 0;  //!< [rad]
    double distance = 0;   //!< [m]
    double closeup = 0;    //!< blend weight: 0 overview, 1 close-up
    double shot = 0;       //!< blend weight of a correction shot
  };

  /** Starts a correction shot of the given region (in the map) at time `t`.
   *  \return The time when the camera is there, or none if ignored (too soon
   *  after the previous one, or during the final overview). */
  std::optional<double> start_shot(double t, const mrpt::math::TBoundingBox & region);

  /// Weight of the current correction shot at time `t` (0: none).
  [[nodiscard]] double shot_weight_at(double t) const;

  /** The camera at time `t` [s] since the start, or none if there is
   *  nothing to look at yet. */
  std::optional<Camera> step(double t, const Inputs & in);

  /// Close-up weight of the schedule at time `t` [s] (0: overview).
  [[nodiscard]] static double closeup_weight_at(const CameraDirectorParams & p, double t);

  /// Distance [m] to see the whole box from above at the given azimuth [rad].
  [[nodiscard]] static double overview_distance(
    const CameraDirectorParams & p, const mrpt::math::TBoundingBox & box, double azimuth);

private:
  CameraDirectorParams params_;
  double last_t_ = 0;
  // Filtered inputs:
  std::optional<double> heading_az_;
  std::optional<mrpt::math::TPoint3D> vehicle_;
  std::optional<mrpt::math::TPoint3D> map_center_;
  std::optional<double> map_log_distance_;
  // Close-up minus overview azimuth, kept continuous during a blend (re-wrapping
  // it would flip its sign, and the camera, when both are nearly opposite):
  std::optional<double> azimuth_diff_;
  // The close-up weight when the final overview started, and when:
  std::optional<std::pair<double, double>> final_start_;

  struct Shot
  {
    double start = 0;
    mrpt::math::TPoint3D center;
    double distance = 0;
  };
  std::optional<Shot> shot_;
};

}  // namespace mola::mapper
