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
 * @file   Parameters.cpp
 * @brief  Parameters for the Mapper3D central 3D SLAM module.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper_3d/Parameters.h>

namespace mola::mapper_3d
{

void Parameters::loadFrom(const mrpt::containers::yaml & cfg)
{
  // Reference frame IDs
  // -----------------------------------------------------
  MCP_LOAD_REQ(cfg, vehicle_frame_name);
  MCP_LOAD_REQ(cfg, reference_frame_name);
  MCP_LOAD_OPT(cfg, enu_frame_name);

  // Kinematic factors and keyframe creation (motion model)
  // -------------------------------------------------------
  MCP_LOAD_REQ(cfg, kinematic_model);
  MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

  MCP_LOAD_OPT(cfg, min_time_difference_to_create_new_frame);
  MCP_LOAD_OPT(cfg, time_between_frames_to_warning);
  MCP_LOAD_OPT(cfg, gnss_nearby_keyframe_stamp_tolerance);
  MCP_LOAD_OPT(cfg, imu_nearby_keyframe_stamp_tolerance);

  MCP_LOAD_OPT(cfg, imu_max_insert_rate_hz);
  MCP_LOAD_OPT(cfg, odometry_max_insert_rate_hz);
  ASSERT_GE_(imu_max_insert_rate_hz, .0);
  ASSERT_GE_(odometry_max_insert_rate_hz, .0);

  MCP_LOAD_OPT(cfg, aggregate_high_rate_into_edges);
  MCP_LOAD_OPT(cfg, sensor_keyframe_min_period);
  ASSERT_GT_(sensor_keyframe_min_period, .0);

  MCP_LOAD_OPT(cfg, keyframe_creation_source);
  MCP_LOAD_OPT(cfg, sensor_clock_min_period_s);
  ASSERT_GT_(sensor_clock_min_period_s, .0);

  MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
  MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);
  MCP_LOAD_OPT(cfg, sigma_integrator_position);
  MCP_LOAD_OPT(cfg, sigma_integrator_orientation);
  MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_linear);
  MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_angular);

  MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
  MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

  MCP_LOAD_OPT(cfg, enforce_planar_motion);

  if (cfg.has("link_first_pose_to_reference_origin_sigma")) {
    const auto strSigma = cfg["link_first_pose_to_reference_origin_sigma"].as<std::string>();
    double sigma = 0;
    if (1 == ::sscanf(strSigma.c_str(), "%lf", &sigma)) {
      ASSERT_GT_(sigma, .0);
      link_first_pose_to_reference_origin_sigma = sigma;
    }
  }

  if (cfg.has("initial_twist")) {
    ASSERT_(cfg["initial_twist"].isSequence() && cfg["initial_twist"].asSequence().size() == 6);
    const auto seq = cfg["initial_twist"].asSequenceRange();
    for (size_t i = 0; i < 6; i++) {
      initial_twist[i] = seq.at(i).as<double>();
    }
  }

  // Keyframe ingestion (SharedKeyframeMap sink)
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, keyframe_ingestion_sigma_lin);
  MCP_LOAD_OPT(cfg, keyframe_ingestion_sigma_ang_deg);
  ASSERT_GT_(keyframe_ingestion_sigma_lin, .0);
  ASSERT_GT_(keyframe_ingestion_sigma_ang_deg, .0);

  // IMU related
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, imu_attitude_sigma_deg);
  MCP_LOAD_OPT(cfg, imu_attitude_azimuth_offset_deg);
  MCP_LOAD_OPT(cfg, imu_normalized_gravity_alignment_sigma);
  MCP_LOAD_OPT(cfg, imu_angular_velocity_sigma_deg);

  // Geo-referencing
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, estimate_geo_reference);
  MCP_LOAD_OPT(cfg, gnss_huber_threshold);
  MCP_LOAD_OPT(cfg, convergence_max_position_sigma);
  MCP_LOAD_OPT(cfg, convergence_max_orientation_sigma_deg);
  MCP_LOAD_OPT(cfg, publish_estimated_georef_on_convergence);

  if (cfg.has("fixed_geo_reference")) {
    auto & gr = fixed_geo_reference.emplace();
    const auto & fgr = cfg["fixed_geo_reference"];

    ASSERT_(fgr.isMap());
    ASSERT_(fgr.has("latitude_deg"));
    ASSERT_(fgr.has("longitude_deg"));
    ASSERT_(fgr.has("altitude"));

    gr.geo_coord.lat = fgr["latitude_deg"].as<double>();
    gr.geo_coord.lon = fgr["longitude_deg"].as<double>();
    gr.geo_coord.height = fgr["altitude"].as<double>();

    gr.T_enu_to_map.cov.setIdentity();
    gr.T_enu_to_map.cov *= 1e-6;
  }

  // Nonlinear optimization
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, additional_isam2_update_steps);
  MCP_LOAD_OPT(cfg, enable_optimizer_thread);

  // High-rate pose publisher
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, high_rate_pose_publish_rate_hz);
  MCP_LOAD_OPT(cfg, high_rate_use_latest_sensors);

  // Map save/load
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, load_simplemap_file);
  MCP_LOAD_OPT(cfg, save_simplemap_file);
  MCP_LOAD_OPT(cfg, externalize_after_seconds);

  // Sensor input names (regex)
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, do_process_imu_labels_re);
  MCP_LOAD_OPT(cfg, do_process_odometry_labels_re);
  MCP_LOAD_OPT(cfg, do_process_gnss_labels_re);

  // Output trajectory
  // -----------------------------------------------------
  MCP_LOAD_OPT(cfg, save_trajectory_to_file);
}

}  // namespace mola::mapper_3d
