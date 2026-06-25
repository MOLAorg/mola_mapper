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

Mapper3D::~Mapper3D() = default;

void Mapper3D::initialize(const mrpt::containers::yaml & cfg)
{
  MRPT_START

  // Load the base NavStateFilter config (subscribes to raw_data_source, etc.):
  mola::NavStateFilter::initialize(cfg);

  ASSERTMSG_(
    cfg.has("params"), "YAML configuration must have a `params` entry with the module options.");

  auto lck = mrpt::lockHelper(stateMutex_);

  params_.loadFrom(cfg["params"]);
  params_loaded_ = true;

  imu_labels_re_ = std::regex(params_.do_process_imu_labels_re);
  odometry_labels_re_ = std::regex(params_.do_process_odometry_labels_re);
  gnss_labels_re_ = std::regex(params_.do_process_gnss_labels_re);

  reset();

  MRPT_LOG_INFO_STREAM(
    "Initialized Mapper3D with reference_frame='"
    << params_.reference_frame_name << "', vehicle_frame='" << params_.vehicle_frame_name << "'");

  MRPT_END
}

void Mapper3D::reset()
{
  auto lck = mrpt::lockHelper(stateMutex_);
  state_.clear();
  reinitialize_gtsam_locked();
}

void Mapper3D::spinOnce()
{
  MRPT_START
  // Nothing periodic yet. Future phases:
  //  - high-rate pose publisher
  //  - background loop closure trigger
  //  - keyframe externalization
  //  - diagnostics publication
  if (module_is_time_to_publish_diagnostics()) {
    // (diagnostics are pulled via getDiagnostics(); nothing to push here yet)
  }
  MRPT_END
}

bool Mapper3D::has_converged_localization(mrpt::poses::CPose3DPDFGaussian & pose_in_map) const
{
  // not implemented yet.
  (void)pose_in_map;
  return false;
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
