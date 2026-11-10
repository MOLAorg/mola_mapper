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
 * @brief  Parameters of mola::mapper::Mapper, loaded from YAML.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Parameters.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>

#include <set>
#include <string>

namespace mola::mapper
{

void GlobalGraphParams::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, odometry_uncertainty_multiplier);
  MCP_LOAD_OPT(cfg, odometry_additional_noise_xyz);
  MCP_LOAD_OPT(cfg, odometry_additional_noise_ang_deg);
  MCP_LOAD_OPT(cfg, odometry_noise_xyz_per_sqrt_m);
  MCP_LOAD_OPT(cfg, odometry_noise_ang_deg_per_sqrt_m);
  MCP_LOAD_OPT(cfg, odometry_noise_ang_per_rad);
  MCP_LOAD_OPT(cfg, use_wheel_odometry);
  MCP_LOAD_OPT(cfg, wheel_odom_min_sigma);
  MCP_LOAD_OPT(cfg, wheel_odom_sigma_per_m);
  MCP_LOAD_OPT(cfg, wheel_odom_lateral_sigma_per_m);
  MCP_LOAD_OPT(cfg, wheel_odom_sigma_per_rad);
  MCP_LOAD_OPT(cfg, wheel_odom_max_yaw_error_deg);
  MCP_LOAD_OPT(cfg, wheel_odom_max_yaw_error_ratio);
  MCP_LOAD_OPT(cfg, wheel_odom_min_ratio_to_front_end);
  MCP_LOAD_OPT(cfg, wheel_odom_ratio_floor);
  MCP_LOAD_OPT(cfg, wheel_odom_robust_kernel);
  MCP_LOAD_OPT(cfg, wheel_odom_robust_k);
  MCP_LOAD_OPT(cfg, wheel_odom_chain_robust_kernel);
  MCP_LOAD_OPT(cfg, wheel_odom_chain_robust_k);
  MCP_LOAD_OPT(cfg, use_gyro);
  MCP_LOAD_OPT(cfg, gyro_min_sigma_deg);

  MCP_LOAD_OPT(cfg, use_gnss);
  MCP_LOAD_OPT(cfg, gnss_minimum_uncertainty);
  MCP_LOAD_OPT(cfg, gnss_uncertainty_multiplier);
  MCP_LOAD_OPT(cfg, gnss_max_uncertainty_horiz);
  MCP_LOAD_OPT(cfg, gnss_robust_kernel);
  MCP_LOAD_OPT(cfg, gnss_robust_k);
  MCP_LOAD_OPT(cfg, georef_min_gnss_factors);
  MCP_LOAD_OPT(cfg, georef_max_sigma_yaw_deg);
  {
    const std::set<std::string> kernels = {"huber", "cauchy", "geman_mcclure", "tukey"};
    ASSERTMSG_(
      kernels.count(gnss_robust_kernel) != 0,
      mrpt::format(
        "Unknown gnss_robust_kernel '%s' (huber, cauchy, geman_mcclure or tukey)",
        gnss_robust_kernel.c_str()));
  }

  MCP_LOAD_OPT(cfg, use_imu_gravity);
  MCP_LOAD_OPT(cfg, imu_gravity_sigma_deg);

  MCP_LOAD_OPT(cfg, loop_closure_robust_huber_k);
  MCP_LOAD_OPT(cfg, loop_closure_min_sigma_xyz);
  MCP_LOAD_OPT(cfg, loop_closure_min_sigma_ang_deg);
  MCP_LOAD_OPT(cfg, first_pose_prior_sigma);
  MCP_LOAD_OPT(cfg, max_iterations);
}

void Parameters::loadFrom(const mrpt::containers::yaml & cfg)
{
  MCP_LOAD_OPT(cfg, reference_frame_name);

  MCP_LOAD_OPT(cfg, gnss_sensor_labels_re);
  MCP_LOAD_OPT(cfg, gnss_max_time_offset);
  MCP_LOAD_OPT(cfg, imu_sensor_labels_re);
  MCP_LOAD_OPT(cfg, imu_gravity_window);
  MCP_LOAD_OPT(cfg, imu_gravity_min_samples);
  MCP_LOAD_OPT(cfg, gyro_sensor_labels_re);
  MCP_LOAD_OPT(cfg, wheel_odom_sensor_labels_re);
  MCP_LOAD_OPT(cfg, wheel_odom_max_gap);
  MCP_LOAD_OPT(cfg, image_sensor_labels_re);
  MCP_LOAD_OPT(cfg, image_max_time_offset);
  MCP_LOAD_OPT(cfg, image_format);
  MCP_LOAD_OPT(cfg, image_jpeg_quality);
  ASSERTMSG_(
    image_format == "jpg" || image_format == "png" || image_format == "inline",
    "image_format must be one of: jpg, png, inline. Got: '" + image_format + "'");
  MCP_LOAD_OPT(cfg, gyro_noise_density);
  MCP_LOAD_OPT(cfg, gyro_bias_sigma);
  MCP_LOAD_OPT(cfg, gyro_scale_sigma);
  MCP_LOAD_OPT(cfg, gyro_max_gap);

  if (cfg.has("graph")) {
    graph.loadFrom(cfg["graph"]);
  }
  MCP_LOAD_OPT(cfg, enable_optimizer_thread);
  MCP_LOAD_OPT(cfg, optimizer_min_period);

  MCP_LOAD_OPT(cfg, feedback_to_state_estimator);
  MCP_LOAD_OPT(cfg, feedback_min_sigma_xyz);
  MCP_LOAD_OPT(cfg, feedback_min_sigma_ang_deg);
  MCP_LOAD_OPT(cfg, local_map_corrections);

  MCP_LOAD_OPT(cfg, loop_closure_enabled);
  MCP_LOAD_OPT(cfg, enable_loop_closure_thread);
  MCP_LOAD_OPT(cfg, loop_closure_pipeline_file);
  MCP_LOAD_OPT(cfg, loop_closure_check_period_sec);
  MCP_LOAD_OPT(cfg, loop_closure_min_new_keyframes);
  MCP_LOAD_OPT(cfg, loop_closure_incremental);
  MCP_LOAD_OPT(cfg, loop_closure_full_scan_every_n);
  MCP_LOAD_OPT(cfg, loop_closure_finalize_rounds);
  MCP_LOAD_OPT(cfg, loop_closure_odometry_check);
  if (cfg.has("loop_closure_odometry_limits")) {
    loop_closure_odometry_limits.loadFrom(cfg["loop_closure_odometry_limits"]);
  }

  MCP_LOAD_OPT(cfg, output_tum_trajectory);
  MCP_LOAD_OPT(cfg, output_simplemap);
  MCP_LOAD_OPT(cfg, output_georeference);
  MCP_LOAD_OPT(cfg, output_loop_closures);
  MCP_LOAD_OPT(cfg, output_state_estimator_tum);
  MCP_LOAD_OPT(cfg, output_state_estimator_georeference);
  MCP_LOAD_OPT(cfg, output_graph_input);
  MCP_LOAD_OPT(cfg, record_input);
}

}  // namespace mola::mapper
