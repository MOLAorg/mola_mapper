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
 * @file   Mapper3D.cpp
 * @brief  Mapper3D lifecycle: initialize, spinOnce, reset, diagnostics.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper_3d/Mapper3D.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Mapper3D, mola::ExecutableBase, mola::mapper_3d)

namespace mola::mapper_3d
{

Mapper3D::Mapper3D()
{
  this->setLoggerName("Mapper3D");
  profiler_.setName("Mapper3D");
  ExecutableBase::setModuleInstanceName("Mapper3D");
}

Mapper3D::~Mapper3D()
{
  stop_optimizer_thread();
  saveEstimatedTrajectoryToFile();
}

void Mapper3D::initialize(const mrpt::containers::yaml & cfg)
{
  MRPT_START

  // Load the base NavStateFilter config (subscribes to raw_data_source, etc.):
  mola::NavStateFilter::initialize(cfg);

  ASSERTMSG_(
    cfg.has("params"), "YAML configuration must have a `params` entry with the module options.");

  // Stop any previously-running optimizer thread BEFORE touching state (must be
  // done with no lock held: the thread itself takes stateMutex_).
  stop_optimizer_thread();

  {
    auto lck = mrpt::lockHelper(stateMutex_);

    params_.loadFrom(cfg["params"]);
    params_loaded_ = true;

    imu_labels_re_ = std::regex(params_.do_process_imu_labels_re);
    odometry_labels_re_ = std::regex(params_.do_process_odometry_labels_re);
    gnss_labels_re_ = std::regex(params_.do_process_gnss_labels_re);

    // Configure the filtered-gravity leveling estimator from params:
    ImuGravityFilter::Parameters gf;
    gf.window_sec = params_.imu_gravity_window_sec;
    gf.accel_tol_frac = params_.imu_gravity_accel_tol_frac;
    gf.gyro_tol_dps = params_.imu_gravity_gyro_tol_deg;
    gf.min_accepted = params_.imu_gravity_min_samples;
    gf.sigma_floor_deg = params_.imu_gravity_sigma_floor_deg;
    gf.sigma_ceil_deg = params_.imu_gravity_sigma_ceil_deg;
    imu_gravity_filter_.setParams(gf);

    // The LocalVelocityBuffer must retain samples for at least one full
    // keyframe interval (it is drained once per keyframe close); otherwise its
    // default 0.5 s pruning would evict the early part of a longer interval.
    imu_buffer_.parameters.max_time_window = std::max(2.0, params_.time_between_frames_to_warning);

    state_.clear();
    reinitialize_gtsam_locked();
    reset_sensor_anchors_locked();
    // Full (re)initialization wipes the map, so the diagnostic counters and the
    // geo-ref-converged announcement reset too (reset() does NOT, see below).
    gnss_factors_inserted_ = 0;
    imu_factors_inserted_ = 0;
    georef_converged_announced_ = false;
  }

  // Optional visualization config + attach to a visualizer module (MolaViz /
  // MolaVizImGui). Both implement mola::VizInterface; absence is fine (headless
  // runs, unit tests).
  if (cfg.has("visualization")) {
    viz_params_ = cfg["visualization"];
  }
  {
    auto viz = findService<VizInterface>();
    if (viz.size() == 1) {
      visualizer_ = std::dynamic_pointer_cast<VizInterface>(viz[0]);
      ASSERT_(visualizer_);
      gui_created_ = false;
      MRPT_LOG_INFO("Attached to a VizInterface module");
    }
  }

  // Start the background optimizer thread (if enabled) AFTER state is ready.
  if (params_.enable_optimizer_thread) {
    optimizer_should_exit_.store(false);
    optimizer_thread_ = std::thread(&Mapper3D::optimizer_thread_loop, this);
  }

  MRPT_LOG_INFO_STREAM(
    "Initialized Mapper3D with reference_frame='"
    << params_.reference_frame_name << "', vehicle_frame='" << params_.vehicle_frame_name
    << "', optimizer_thread=" << (params_.enable_optimizer_thread ? "on" : "off"));

  MRPT_END
}

void Mapper3D::optimizer_thread_loop()
{
  while (!optimizer_should_exit_.load()) {
    {
      std::unique_lock<std::mutex> lk(optimizer_wakeup_mutex_);
      // Wake on new data, on shutdown, or periodically (a safety net so a
      // missed notification cannot stall the backend indefinitely).
      optimizer_wakeup_cv_.wait_for(lk, std::chrono::milliseconds(50), [this] {
        return optimizer_pending_ || optimizer_should_exit_.load();
      });
      optimizer_pending_ = false;
    }
    if (optimizer_should_exit_.load()) {
      break;
    }
    try {
      optimize_and_refresh();
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("[optimizer thread] " << e.what());
    }
  }
}

void Mapper3D::notify_optimizer()
{
  if (!params_.enable_optimizer_thread) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(optimizer_wakeup_mutex_);
    optimizer_pending_ = true;
  }
  optimizer_wakeup_cv_.notify_one();
}

void Mapper3D::stop_optimizer_thread()
{
  optimizer_should_exit_.store(true);
  optimizer_wakeup_cv_.notify_all();
  if (optimizer_thread_.joinable()) {
    optimizer_thread_.join();
  }
  optimizer_should_exit_.store(false);
}

void Mapper3D::saveEstimatedTrajectoryToFile()
{
  if (!params_loaded_ || params_.save_trajectory_to_file.empty()) {
    return;
  }

  // Snapshot the trajectory under the state lock.
  std::vector<std::pair<double, mrpt::poses::CPose3D>> traj;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    for (const auto & [t, kfId] : state_.time_to_kf_id.getDirectMap()) {
      const auto it = state_.last_estimated_states.find(kfId);
      if (it == state_.last_estimated_states.end()) {
        continue;
      }
      traj.emplace_back(mrpt::Clock::toDouble(t), it->second.pose);
    }
  }

  const auto & fil = params_.save_trajectory_to_file;
  MRPT_LOG_INFO_STREAM(
    "Saving estimated trajectory with " << traj.size() << " keyframes to '" << fil
                                        << "' in TUM format (frame: '"
                                        << params_.reference_frame_name << "')...");

  std::ofstream f(fil);
  if (!f.is_open()) {
    MRPT_LOG_ERROR_STREAM("Cannot open file for writing: " << fil);
    return;
  }

  f << std::fixed << std::setprecision(6);
  for (const auto & [t, pose] : traj) {
    mrpt::math::CQuaternionDouble q;
    pose.getAsQuaternion(q);
    // TUM format: timestamp tx ty tz qx qy qz qw
    f << t << " " << pose.x() << " " << pose.y() << " " << pose.z() << " " << q.x() << " " << q.y()
      << " " << q.z() << " " << q.r() << "\n";
  }
  MRPT_LOG_INFO("Estimated trajectory saved.");
}

void Mapper3D::reset()
{
  auto lck = mrpt::lockHelper(stateMutex_);
  // A reset() request (e.g. a front end re-localizing, like LidarOdometry's
  // startup `navstate_fuse->reset()`) means "forget the short-term per-source
  // integration state", NOT "wipe the central world model". The keyframes,
  // factor graph and geo-referencing are the persistent, shared map: they MUST
  // survive a single front end's relocalization, otherwise LIO's startup reset
  // would erase the IMU/GNSS keyframes + tentative geo-reference Mapper3D
  // accumulated during LIO's warmup (and geo-ref would converge, get wiped, then
  // re-converge). So only the per-source high-rate integration anchors and the
  // keyframe-ingestion chains are reset, so each source re-anchors cleanly; the
  // map, the diagnostic counters and the geo-ref-converged announcement persist.
  reset_sensor_anchors_locked();
}

void Mapper3D::reset_sensor_anchors_locked()
{
  last_wheels_odometry_.reset();
  last_wheels_odometry_name_.reset();
  last_wheels_odometry_stamp_.reset();
  imu_buffer_.clear();
  imu_transformers_.clear();
  last_imu_kf_.reset();
  imu_gravity_filter_.clear();
  wheel_chain_last_kf_.reset();
  wheel_chain_anchor_odom_.reset();
  prev_shared_kf_id_.reset();
  wheel_odom_at_prev_shared_kf_.reset();
  kf_odom_abs_pose_.clear();
  odom_chain_edges_.clear();
  odom_frame_anchored_.clear();
  latest_kf_by_odom_frame_.clear();
  last_publish_wallclock_.reset();
}

void Mapper3D::spinOnce()
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "spinOnce");
  // High-rate extrapolated pose publication. The heavy iSAM2 solve runs on the
  // optimizer thread (when enabled), so this stays cheap: read the latest
  // committed anchor + extrapolate.
  publish_high_rate_pose();
  // Refresh the 3D scene + GUI (throttled, no-op without a visualizer).
  updateVisualization();
  // Future phases: background loop-closure trigger, keyframe externalization.
  MRPT_END
}

void Mapper3D::publish_high_rate_pose()
{
  if (params_.high_rate_pose_publish_rate_hz <= 0) {
    return;
  }
  if (!anyUpdateLocalizationSubscriber()) {
    return;
  }

  // Throttle to the configured rate using wall-clock time:
  const auto nowWall = mrpt::Clock::now();
  if (last_publish_wallclock_.has_value()) {
    const double dt = mrpt::system::timeDifference(*last_publish_wallclock_, nowWall);
    if (dt < 1.0 / params_.high_rate_pose_publish_rate_hz) {
      return;
    }
  }

  // Publish at the latest fused keyframe time (the freshest data instant):
  std::optional<mrpt::Clock::time_point> stamp;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    if (!state_.time_to_kf_id.empty()) {
      stamp = state_.time_to_kf_id.getDirectMap().rbegin()->first;
    }
  }
  if (!stamp.has_value()) {
    return;
  }

  const auto nv = estimated_navstate(*stamp, params_.reference_frame_name);
  if (!nv) {
    MRPT_LOG_THROTTLE_WARN(5.0, "[publish] Cannot publish pose yet (no usable estimate).");
    return;
  }
  last_publish_wallclock_ = nowWall;

  LocalizationUpdate lu;
  lu.child_frame = params_.vehicle_frame_name;
  lu.reference_frame = params_.reference_frame_name;
  const auto & fullName = getModuleInstanceName();
  const auto colonPos = fullName.rfind(':');
  lu.method = (colonPos != std::string::npos) ? fullName.substr(colonPos + 1) : fullName;
  lu.quality = 1;
  lu.timestamp = *stamp;
  lu.pose = nv->pose.getPoseMean().asTPose();
  lu.cov = nv->pose.cov_inv.inverse_LLt();

  advertiseUpdatedLocalization(lu);
}

bool Mapper3D::has_converged_localization(mrpt::poses::CPose3DPDFGaussian & pose_in_map) const
{
  auto lck = mrpt::lockHelper(stateMutex_);

  // Converged if we were told to estimate geo-referencing, and the
  // convergence-gated publication in optimize_and_refresh()
  // has already published one (see that function for the actual sigma
  // thresholds against convergence_max_position_sigma /
  // convergence_max_orientation_sigma_deg).
  const bool converged = params_.estimate_geo_reference && state_.geo_reference.has_value();

  if (converged && !state_.time_to_kf_id.empty()) {
    const NavState ns = get_latest_state_and_covariance(state_.last_kf_id());
    pose_in_map.copyFrom(ns.pose);
  }

  return converged;
}

void Mapper3D::getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status)
{
  auto lck = mrpt::lockHelper(stateMutex_);

  mola::DiagnosticStatusMsg msg;
  msg.name = "Mapper3D";
  msg.level = mola::DiagnosticLevel::OK;
  msg.message = "running";
  msg.values.push_back({"keyframes", std::to_string(state_.time_to_kf_id.size())});
  msg.values.push_back({"odometry_frames", std::to_string(state_.known_odom_frames.size())});
  msg.values.push_back({"geo_referenced", state_.geo_reference.has_value() ? "yes" : "no"});
  msg.values.push_back({"gnss_factors", std::to_string(gnss_factors_inserted_)});
  msg.values.push_back({"imu_factors", std::to_string(imu_factors_inserted_)});

  // Geo-referencing transform + per-source odometry-frame drift vs {map}.
  if (const auto itEnu = state_.last_estimated_frames.find(0);
      itEnu != state_.last_estimated_frames.end()) {
    const auto & m = itEnu->second.mean;
    msg.values.push_back(
      {"T_enu_to_map",
       mrpt::format(
         "(%.2f, %.2f, %.2f, yaw=%.2fdeg)", m.x(), m.y(), m.z(), mrpt::RAD2DEG(m.yaw()))});
  }
  for (const auto & [name, id] : state_.known_odom_frames.getDirectMap()) {
    const auto itF = state_.last_estimated_frames.find(id);
    if (itF == state_.last_estimated_frames.end()) {
      continue;
    }
    // T_map_to_odom_i as translation + rotation magnitude (LIO drift is mostly
    // z/tilt, so the rotation term is the informative one; see Mapper3D_GUI).
    const double cosAngle =
      std::clamp((itF->second.mean.getRotationMatrix().trace() - 1.0) * 0.5, -1.0, 1.0);
    msg.values.push_back(
      {"drift_" + name, mrpt::format(
                          "%.2fm/%.1fdeg", itF->second.mean.translation().norm(),
                          mrpt::RAD2DEG(std::acos(cosAngle)))});
  }

  status.push_back(std::move(msg));
}

std::set<std::string> Mapper3D::known_odometry_frame_ids() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  std::set<std::string> ret;
  for (const auto & [name, id] : state_.known_odom_frames.getDirectMap()) {
    (void)id;
    ret.insert(name);
  }
  return ret;
}

std::size_t Mapper3D::keyframe_count() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  return state_.time_to_kf_id.size();
}

}  // namespace mola::mapper_3d
