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
 * @file   CameraDirector.cpp
 * @brief  Scripted GUI camera: map overviews and close-ups of the vehicle.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/CameraDirector.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/wrap2pi.h>

#include <algorithm>
#include <cmath>

namespace mola::mapper
{
namespace
{
/// Smooth step from 0 to 1 for u in [0,1] (cosine easing).
double ease(double u) { return 0.5 * (1.0 - std::cos(M_PI * std::clamp(u, 0.0, 1.0))); }

/// Low-pass filter gain for a time step dt and a time constant tau.
double gain(double dt, double tau) { return 1.0 - std::exp(-dt / std::max(tau, 1e-3)); }
}  // namespace

void CameraDirectorParams::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, enabled);
  MCP_LOAD_OPT(cfg, rate);
  MCP_LOAD_OPT(cfg, overview_elevation);
  MCP_LOAD_OPT(cfg, overview_azimuth);
  MCP_LOAD_OPT(cfg, overview_margin);
  MCP_LOAD_OPT(cfg, overview_min_distance);
  MCP_LOAD_OPT(cfg, closeup_elevation);
  MCP_LOAD_OPT(cfg, closeup_distance);
  MCP_LOAD_OPT(cfg, overview_period);
  MCP_LOAD_OPT(cfg, closeup_period);
  MCP_LOAD_OPT(cfg, transition);
  MCP_LOAD_OPT(cfg, heading_time_constant);
  MCP_LOAD_OPT(cfg, vehicle_time_constant);
  MCP_LOAD_OPT(cfg, map_time_constant);
  MCP_LOAD_OPT(cfg, fov);
  MCP_LOAD_OPT(cfg, aspect_ratio);
  MCP_LOAD_OPT(cfg, final_overview);
}

double CameraDirector::closeup_weight_at(const CameraDirectorParams & p, double t)
{
  const double ov = std::max(p.overview_period, 0.0);
  const double cu = std::max(p.closeup_period, 0.0);
  const double tr = std::max(p.transition, 1e-3);
  const double tIn = std::fmod(std::max(t, 0.0), ov + cu + 2 * tr);
  if (tIn < ov) {
    return 0.0;
  }
  if (tIn < ov + tr) {
    return ease((tIn - ov) / tr);
  }
  if (tIn < ov + tr + cu) {
    return 1.0;
  }
  return 1.0 - ease((tIn - ov - tr - cu) / tr);
}

double CameraDirector::overview_distance(
  const CameraDirectorParams & p, const mrpt::math::TBoundingBox & box, double azimuth)
{
  // Seen from above, the screen "up" axis points away from the camera azimuth.
  const double ux = std::abs(std::cos(azimuth));
  const double uy = std::abs(std::sin(azimuth));
  const double hx = 0.5 * (box.max.x - box.min.x) + p.overview_margin;
  const double hy = 0.5 * (box.max.y - box.min.y) + p.overview_margin;
  // Half extents of the box along the screen up and right axes:
  const double halfUp = ux * hx + uy * hy;
  const double halfRight = uy * hx + ux * hy;
  const double halfHeight = std::max(halfUp, halfRight / std::max(p.aspect_ratio, 0.1));
  const double d = halfHeight / std::tan(0.5 * mrpt::DEG2RAD(std::clamp(p.fov, 1.0, 170.0)));
  return std::max(d, p.overview_min_distance);
}

std::optional<CameraDirector::Camera> CameraDirector::step(double t, const Inputs & in)
{
  const double dt = std::max(t - last_t_, 0.0);
  last_t_ = t;

  // Close-up: behind the vehicle.
  if (in.vehicle_yaw) {
    const double target = mrpt::math::wrapToPi(*in.vehicle_yaw + M_PI);
    if (!heading_az_) {
      heading_az_ = target;
    } else {
      heading_az_ = mrpt::math::wrapToPi(
        *heading_az_ +
        gain(dt, params_.heading_time_constant) * mrpt::math::wrapToPi(target - *heading_az_));
    }
  }
  if (in.vehicle) {
    vehicle_ = vehicle_
                 ? *vehicle_ + (*in.vehicle - *vehicle_) * gain(dt, params_.vehicle_time_constant)
                 : *in.vehicle;
  }

  // Overview: the whole map.
  const double ovAzimuth = mrpt::DEG2RAD(params_.overview_azimuth);
  if (in.map) {
    const auto center = (in.map->min + in.map->max) * 0.5;
    const double logDist = std::log(overview_distance(params_, *in.map, ovAzimuth));
    const double k = gain(dt, params_.map_time_constant);
    map_center_ = map_center_ ? *map_center_ + (center - *map_center_) * k : center;
    map_log_distance_ =
      map_log_distance_ ? *map_log_distance_ + (logDist - *map_log_distance_) * k : logDist;
  }

  // The blend weight, from the schedule, then down to the overview at the end:
  double w = closeup_weight_at(params_, t);
  if (in.final && !final_start_) {
    final_start_ = {w, t};
  }
  if (final_start_) {
    const auto [w0, t0] = *final_start_;
    w = w0 * (1.0 - ease((t - t0) / std::max(params_.transition, 1e-3)));
  }
  if (!vehicle_ && !map_center_) {
    return std::nullopt;
  }
  if (!vehicle_) {
    w = 0;
  } else if (!map_center_) {
    w = 1;
  }

  const double cuAzimuth = heading_az_.value_or(ovAzimuth);
  const auto ovCenter = map_center_.value_or(mrpt::math::TPoint3D(0, 0, 0));
  const double ovLogDist = map_log_distance_.value_or(std::log(params_.overview_min_distance));
  const auto cuCenter = vehicle_.value_or(ovCenter);
  const double cuLogDist = std::log(std::max(params_.closeup_distance, 1e-3));

  Camera c;
  c.closeup = w;
  c.look_at = ovCenter + (cuCenter - ovCenter) * w;
  c.azimuth = mrpt::math::wrapToPi(ovAzimuth + w * mrpt::math::wrapToPi(cuAzimuth - ovAzimuth));
  c.elevation = mrpt::DEG2RAD(
    params_.overview_elevation + w * (params_.closeup_elevation - params_.overview_elevation));
  c.distance = std::exp(ovLogDist + w * (cuLogDist - ovLogDist));
  return c;
}

}  // namespace mola::mapper
