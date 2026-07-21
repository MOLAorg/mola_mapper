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
 * @file   Mapper.cpp
 * @brief  Mapper lifecycle: initialize, spinOnce, reset, diagnostics.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <string>

#include "covariance_utils.h"

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Mapper, mola::ExecutableBase, mola::mapper)

namespace mola::mapper
{

Mapper::Mapper()
{
  this->setLoggerName("Mapper");
  profiler_.setName("Mapper");
  ExecutableBase::setModuleInstanceName("Mapper");
}

Mapper::~Mapper()
{
  // Batch loop-closure pass over the now-complete trajectory (stops the LC
  // thread internally, then runs full scans + re-optimization to convergence).
  try {
    finalize_loop_closures();
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("[loop_closure] finalize failed: " << e.what());
  }

  // Stop the LC thread first: it feeds the optimizer, so it must not enqueue
  // more work once the optimizer is gone.
  stop_loop_closure_thread();
  stop_optimizer_thread();
  saveEstimatedTrajectoryToFile();
  saveSimpleMapToFile();
}

void Mapper::initialize(const mrpt::containers::yaml & cfg)
{
  MRPT_START

  // Load the base NavStateFilter config (subscribes to raw_data_source, etc.):
  mola::NavStateFilter::initialize(cfg);

  ASSERTMSG_(
    cfg.has("params"), "YAML configuration must have a `params` entry with the module options.");

  // Stop any previously-running background threads BEFORE touching state (must
  // be done with no lock held: they take stateMutex_). LC first (it feeds the
  // optimizer).
  stop_loop_closure_thread();
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
    gf.max_spread_deg = params_.imu_gravity_max_spread_deg;
    gf.min_accept_fraction = params_.imu_gravity_min_accept_fraction;
    gf.sigma_floor_deg = params_.imu_gravity_sigma_floor_deg;
    gf.sigma_ceil_deg = params_.imu_gravity_sigma_ceil_deg;
    imu_gravity_filter_.setParams(gf);

    // The LocalVelocityBuffer must retain enough history to cover BOTH a full
    // keyframe interval and the (possibly longer) gravity-leveling look-back
    // window, which is drained on keyframe close; otherwise its default 0.5 s
    // pruning would evict the early part of a longer window.
    double imuRetention = std::max(
      std::max(2.0, params_.time_between_frames_to_warning), params_.imu_gravity_window_sec + 0.5);
    if (params_.imu_relative_rotation_enabled || params_.imu_preintegration_enabled) {
      // These integrate the FULL inter-keyframe interval, so a truncated buffer
      // silently yields a delta covering only part of the gap (which then gets
      // attributed to the whole interval, corrupting the constraint). On a
      // distance-gated central map that interval is many seconds, far more than
      // the warning threshold above. Retaining a generous window is cheap
      // (a few hundred Hz x tens of seconds is a small map).
      imuRetention = std::max(imuRetention, params_.imu_integration_buffer_retention_sec);
    }
    imu_buffer_.parameters.max_time_window = imuRetention;

    state_.clear();
    reinitialize_gtsam_locked();
    reset_sensor_anchors_locked();
    // Full (re)initialization wipes the map, so the diagnostic counters and the
    // geo-ref-converged announcement reset too (reset() does NOT, see below).
    geo_ref_counters_ = {};
  }

  // Optional visualization config + attach to a visualizer module (MolaViz /
  // MolaVizImGui). Both implement mola::VizInterface; absence is fine (headless
  // runs, unit tests).
  if (cfg.has("visualization")) {
    viz_params_ = cfg["visualization"];
    viz_keyframe_corner_scale_.store(viz_params_.getOrDefault<float>("keyframe_corner_size", 0.5f));
    viz_keyframe_sphere_radius_.store(
      viz_params_.getOrDefault<float>("keyframe_sphere_radius", 0.2f));
    viz_edge_cylinder_radius_.store(viz_params_.getOrDefault<float>("edge_cylinder_radius", 0.05f));
    viz_camera_follows_vehicle_.store(
      viz_params_.getOrDefault<bool>("camera_follows_vehicle", viz_camera_follows_vehicle_.load()));
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
    optimizer_thread_ = std::thread(&Mapper::optimizer_thread_loop, this);
  }

  // Start the background loop-closure thread (if enabled) AFTER the optimizer,
  // so the edges it merges have somewhere to be solved.
  start_loop_closure_thread();

  MRPT_LOG_INFO_STREAM(
    "Initialized Mapper with reference_frame='"
    << params_.reference_frame_name << "', vehicle_frame='" << params_.vehicle_frame_name
    << "', optimizer_thread=" << (params_.enable_optimizer_thread ? "on" : "off")
    << ", loop_closure=" << (params_.loop_closure_enabled ? "on" : "off"));

  MRPT_END
}

void Mapper::optimizer_thread_loop()
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

void Mapper::notify_optimizer()
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

void Mapper::stop_optimizer_thread()
{
  optimizer_should_exit_.store(true);
  optimizer_wakeup_cv_.notify_all();
  if (optimizer_thread_.joinable()) {
    optimizer_thread_.join();
  }
  optimizer_should_exit_.store(false);
}

void Mapper::saveEstimatedTrajectoryToFile()
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

void Mapper::saveSimpleMapToFile()
{
  if (!params_loaded_ || params_.save_simplemap_file.empty()) {
    return;
  }

  mrpt::maps::CSimpleMap sm;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    sm = state_.as_simple_map();
  }

  const auto & fil = params_.save_simplemap_file;
  MRPT_LOG_INFO_STREAM("Saving simplemap with " << sm.size() << " keyframes to '" << fil << "'...");

  std::vector<std::future<void>> pendingDiskIO;

  if (params_.generate_lazy_load_scan_files) {
    // Create the default "_Images" directory alongside the output file and
    // externally serialize each keyframe's point clouds there, mirroring
    // mola::LidarOdometry's generate_lazy_load_scan_files feature so the
    // saved simplemap stays small and loads/processes fast downstream.
    const std::string out_basedir = mrpt::system::pathJoin(
      {mrpt::system::extractFileDirectory(fil), mrpt::system::extractFileName(fil) + "_Images"});

    if (!mrpt::system::directoryExists(out_basedir)) {
      const bool dirCreatedOk = mrpt::system::createDirectory(out_basedir);
      ASSERTMSG_(
        dirCreatedOk,
        mrpt::format(
          "Error creating lazy-load directory for output simplemap: '%s'", out_basedir.c_str()));
      MRPT_LOG_INFO_STREAM("Creating lazy-load directory for output .simplemap: " << out_basedir);
    }

    mrpt::io::setLazyLoadPathBase(out_basedir);

    // The actual disk write (unload()) runs on a background worker thread so
    // this method does not block its caller (destructor or GUI thread) on
    // potentially GBs of point cloud I/O. Only the (cheap) externalization
    // flag/filename are set here, synchronously; sm.saveToFile() below only
    // reads those, never the raw point buffer, so it is safe to run
    // concurrently with the pending unload() disk writes. We still wait for
    // all of them below before returning, so the simplemap is fully
    // persisted (bin files included) by the time this method returns.
    for (auto & kf : sm) {
      if (!kf.sf) {
        continue;
      }
      for (auto & obs : *kf.sf) {
        auto oPts = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obs);
        if (!oPts || oPts->isExternallyStored()) {
          continue;
        }
        ASSERT_(oPts->pointcloud);
        const std::string pcFilename = mrpt::format(
          "%s_%.09f.bin", mrpt::system::fileNameStripInvalidChars(oPts->sensorLabel).c_str(),
          mrpt::Clock::toDouble(oPts->timestamp));

        oPts->setAsExternalStorage(
          pcFilename, mrpt::obs::CObservationPointCloud::ExternalStorageFormat::MRPT_Serialization);

        pendingDiskIO.emplace_back(worker_disk_io_.enqueue([oPts]() {
          try {
            oPts->unload();
          } catch (const std::exception & e) {
            std::cerr << "[Mapper] saveSimpleMapToFile(): Error saving observation to disk: "
                      << e.what() << "\n";
          }
        }));
      }
    }
  }

  const bool saveOk = sm.saveToFile(fil);

  // Wait for all queued point cloud writes to finish, so the simplemap
  // (with its externalized .bin files) is fully persisted to disk before
  // this method returns.
  for (auto & fut : pendingDiskIO) {
    fut.wait();
  }

  if (!saveOk) {
    MRPT_LOG_ERROR_STREAM("Error saving simplemap to: " << fil);
    return;
  }
  MRPT_LOG_INFO("Simplemap saved.");
}

void Mapper::reset()
{
  auto lck = mrpt::lockHelper(stateMutex_);
  // A reset() request (e.g. a front end re-localizing, like LidarOdometry's
  // startup `navstate_fuse->reset()`) means "forget the short-term per-source
  // integration state", NOT "wipe the central world model". The keyframes,
  // factor graph and geo-referencing are the persistent, shared map: they MUST
  // survive a single front end's relocalization, otherwise LIO's startup reset
  // would erase the IMU/GNSS keyframes + tentative geo-reference Mapper
  // accumulated during LIO's warmup (and geo-ref would converge, get wiped, then
  // re-converge). So only the per-source high-rate integration anchors and the
  // keyframe-ingestion chains are reset, so each source re-anchors cleanly; the
  // map, the diagnostic counters and the geo-ref-converged announcement persist.
  reset_sensor_anchors_locked();
}

void Mapper::reset_sensor_anchors_locked()
{
  last_wheels_odometry_.reset();
  last_wheels_odometry_name_.reset();
  last_wheels_odometry_stamp_.reset();
  imu_buffer_.clear();
  imu_transformers_.clear();
  last_imu_kf_.reset();
  last_gravity_check_t_.reset();
  last_gravity_kf_id_.reset();
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

void Mapper::spinOnce()
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

void Mapper::publish_high_rate_pose()
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

  // Publish at the freshest data instant (advances between keyframes as dense
  // odometry arrives), not just at the latest keyframe stamp:
  std::optional<mrpt::Clock::time_point> stamp;
  {
    auto lck = mrpt::lockHelper(stateMutex_);
    stamp = get_current_extrapolated_stamp_locked();
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

void Mapper::note_observation_stamp(const mrpt::Clock::time_point & t)
{
  const int64_t ticks = t.time_since_epoch().count();
  int64_t prev = last_observation_stamp_ticks_.load(std::memory_order_relaxed);
  while (ticks > prev && !last_observation_stamp_ticks_.compare_exchange_weak(
                           prev, ticks, std::memory_order_relaxed)) {
    // CAS retried; `prev` refreshed by compare_exchange_weak.
  }
}

std::optional<mrpt::Clock::time_point> Mapper::get_current_extrapolated_stamp_locked() const
{
  std::optional<mrpt::Clock::time_point> stamp;
  // Newest raw odometry anchor (updated every dense fuse_pose(), i.e. at scan
  // rate, well above the keyframe cadence):
  for (const auto & [id, raw] : state_.last_raw_pose_by_source) {
    (void)id;
    if (!stamp.has_value() || raw.stamp > *stamp) {
      stamp = raw.stamp;
    }
  }
  // Newest high-rate raw observation (IMU/wheels/GNSS). In a
  // LIO+SharedKeyframeMap setup these are the mapper's ONLY dense inputs, so
  // this is what advances the extrapolation instant between the sparse
  // keyframes (fuse_pose() odometry sources are usually absent there):
  const int64_t obsTicks = last_observation_stamp_ticks_.load(std::memory_order_relaxed);
  if (obsTicks != kNoObsStamp) {
    const mrpt::Clock::time_point obsStamp{mrpt::Clock::duration{obsTicks}};
    if (!stamp.has_value() || obsStamp > *stamp) {
      stamp = obsStamp;
    }
  }
  // Fall back to (or advance past) the newest keyframe stamp:
  if (!state_.time_to_kf_id.empty()) {
    const auto kfStamp = state_.time_to_kf_id.getDirectMap().rbegin()->first;
    if (!stamp.has_value() || kfStamp > *stamp) {
      stamp = kfStamp;
    }
  }
  return stamp;
}

bool Mapper::has_converged_localization(mrpt::poses::CPose3DPDFGaussian & pose_in_map) const
{
  auto lck = mrpt::lockHelper(stateMutex_);

  if (state_.time_to_kf_id.empty() || state_.last_estimated_states.empty()) {
    return false;
  }

  // A geo-reference is required before a pose in {map} is even meaningful:
  // either already estimated live (estimate_geo_reference=true) or fixed from a
  // loaded geo-referenced map (relocalize mode, via set_geo_reference() or the
  // fixed_geo_reference param).
  if (!state_.geo_reference.has_value()) {
    return false;
  }

  NavState ns;
  try {
    ns = get_latest_state_and_covariance(state_.last_kf_id());
  } catch (const std::exception & e) {
    // Latest keyframe not solved yet: not converged, not a fatal error.
    MRPT_LOG_DEBUG_STREAM("[has_converged_localization] Latest KF state not ready: " << e.what());
    return false;
  }

  bool converged = false;
  if (params_.estimate_geo_reference) {
    // Live geo-referencing: state_.geo_reference is only ever populated by
    // optimize_and_refresh() once T_enu_to_map's OWN sigmas already cleared
    // these same thresholds, and is sticky afterwards (a temporarily worse
    // per-keyframe sigma must not un-converge an established geo-reference).
    converged = true;
  } else {
    // Relocalize mode: the geo-reference is fixed up front, before any GNSS/IMU
    // fusion has happened, so its mere presence says nothing about whether the
    // vehicle is actually localized. Gate on the vehicle's OWN latest pose
    // uncertainty instead.
    const auto [posSigma, oriSigmaDeg] =
      max_pos_and_orientation_sigma(ns.pose.cov_inv.inverse_LLt());

    converged = posSigma <= params_.convergence_max_position_sigma &&
                oriSigmaDeg <= params_.convergence_max_orientation_sigma_deg;

    MRPT_LOG_THROTTLE_DEBUG_FMT(
      2.0,
      "[has_converged_localization] relocalize mode: pos_sigma=%.3f m ori_sigma=%.3f deg "
      "thresh=(%.2f m, %.2f deg) -> %s",
      posSigma, oriSigmaDeg, params_.convergence_max_position_sigma,
      params_.convergence_max_orientation_sigma_deg, converged ? "CONVERGED" : "not yet");
  }

  if (converged) {
    pose_in_map.copyFrom(ns.pose);
  }

  return converged;
}

void Mapper::getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status)
{
  auto lck = mrpt::lockHelper(stateMutex_);

  mola::DiagnosticStatusMsg msg;
  msg.name = "Mapper";
  msg.level = mola::DiagnosticLevel::OK;
  msg.message = "running";
  msg.values.push_back({"keyframes", std::to_string(state_.time_to_kf_id.size())});
  msg.values.push_back({"odometry_frames", std::to_string(state_.known_odom_frames.size())});
  msg.values.push_back({"geo_referenced", state_.geo_reference.has_value() ? "yes" : "no"});
  msg.values.push_back({"gnss_factors", std::to_string(geo_ref_counters_.gnss)});
  msg.values.push_back(
    {"imu_factors", std::to_string(
                      geo_ref_counters_.imu_attitude + geo_ref_counters_.imu_gravity +
                      geo_ref_counters_.imu_omega)});

  // Loop-closure counters (present when the feature is enabled).
  if (params_.loop_closure_enabled) {
    msg.values.push_back({"lc_loops_accepted", std::to_string(lc_ui_.loops_accepted.load())});
    msg.values.push_back(
      {"lc_candidates_checked", std::to_string(lc_ui_.candidates_checked.load())});
    msg.values.push_back({"lc_scans", std::to_string(lc_ui_.scans_completed.load())});
    msg.values.push_back({"lc_scan_in_progress", lc_ui_.scan_in_progress.load() ? "yes" : "no"});
    const std::size_t total = lc_ui_.cur_total.load();
    const std::size_t done = lc_ui_.cur_done.load();
    msg.values.push_back(
      {"lc_queue_depth", std::to_string(total > done ? total - done : std::size_t{0})});
    if (lc_ui_.finalize_active.load()) {
      msg.values.push_back(
        {"lc_finalize_round",
         mrpt::format(
           "%zu/%zu", lc_ui_.finalize_round.load(), lc_ui_.finalize_rounds_total.load())});
    }
  }

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
    // z/tilt, so the rotation term is the informative one; see Mapper_GUI).
    const double cosAngle =
      std::clamp((itF->second.mean.getRotationMatrix().trace() - 1.0) * 0.5, -1.0, 1.0);
    msg.values.push_back(
      {"drift_" + name, mrpt::format(
                          "%.2fm/%.1fdeg", itF->second.mean.translation().norm(),
                          mrpt::RAD2DEG(std::acos(cosAngle)))});
  }

  status.push_back(std::move(msg));
}

std::set<std::string> Mapper::known_odometry_frame_ids() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  std::set<std::string> ret;
  for (const auto & [name, id] : state_.known_odom_frames.getDirectMap()) {
    (void)id;
    ret.insert(name);
  }
  return ret;
}

std::size_t Mapper::keyframe_count() const
{
  auto lck = mrpt::lockHelper(stateMutex_);
  return state_.time_to_kf_id.size();
}

}  // namespace mola::mapper
