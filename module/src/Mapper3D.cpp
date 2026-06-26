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
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/datetime.h>

#include <chrono>
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

Mapper3D::~Mapper3D() { stop_optimizer_thread(); }

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

    state_.clear();
    reinitialize_gtsam_locked();
    reset_sensor_anchors_locked();
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

void Mapper3D::reset()
{
  auto lck = mrpt::lockHelper(stateMutex_);
  state_.clear();
  reinitialize_gtsam_locked();
  reset_sensor_anchors_locked();
}

void Mapper3D::reset_sensor_anchors_locked()
{
  last_wheels_odometry_.reset();
  last_wheels_odometry_name_.reset();
  last_wheels_odometry_stamp_.reset();
  last_processed_imu_stamp_.reset();
  wheel_chain_last_kf_.reset();
  wheel_chain_anchor_odom_.reset();
  keyframe_ingestion_state_by_source_.clear();
  last_publish_wallclock_.reset();
}

void Mapper3D::spinOnce()
{
  MRPT_START
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
