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
 * @file   KeyframeMapVisualization.cpp
 * @brief  GUI map built from the keyframe clouds at their optimized poses.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/KeyframeMapVisualization.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/sm2mm.h>
#include <mrpt/core/format.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <algorithm>
#include <chrono>

namespace mola::mapper
{
namespace
{
// sm2mm pipeline of one keyframe, at the identity pose (the cloud stays in the
// keyframe frame): de-skewed (IMU velocity buffer of the keyframe), points in
// a range of the vehicle, decimated, in one layer "points".
constexpr const char * PIPELINE = R"yaml(
generators:
  - class_name: mp2p_icp_filters::Generator
    params:
      target_layer: "raw"
      throw_on_unhandled_observation_class: false
      process_class_names_regex: "(mrpt::obs::CObservationPointCloud|mrpt::obs::CObservationVelodyneScan)"
      process_sensor_labels_regex: ".*"
filters:
  - class_name: mp2p_icp_filters::FilterDeskew
    params:
      input_pointcloud_layer: "raw"
      output_pointcloud_layer: "deskewed"
      method: "MotionCompensationMethod::IMU"
      silently_ignore_no_timestamps: true
      output_layer_class: "mrpt::maps::CSimplePointsMap"
      twist: [vx, vy, vz, wx, wy, wz]
  - class_name: mp2p_icp_filters::FilterByRange
    params:
      input_pointcloud_layer: "deskewed"
      output_layer_between: "in_range"
      range_min: %f
      range_max: %f
  - class_name: mp2p_icp_filters::FilterDecimateVoxels
    params:
      input_pointcloud_layer: "in_range"
      output_pointcloud_layer: "points"
      voxel_filter_resolution: %f
      decimate_method: DecimateMethod::ClosestToAverage
  - class_name: mp2p_icp_filters::FilterDeleteLayer
    params:
      pointcloud_layer_to_remove: ["raw", "deskewed", "in_range"]
)yaml";
}  // namespace

mrpt::img::TColor turbo_color(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  const auto f = [](double v) { return static_cast<uint8_t>(std::clamp(v, 0.0, 1.0) * 255.0); };
  const double r =
    0.13572138 +
    x * (4.61539260 +
         x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
  const double g =
    0.09140261 +
    x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
  const double b =
    0.10667330 +
    x * (12.64194608 +
         x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
  return {f(r), f(g), f(b), 0xff};
}

KeyframeMapVisualization::KeyframeMapVisualization(const KeyframeMapParams & p)
: params_(p),
  pipeline_(
    mrpt::containers::yaml::FromText(mrpt::format(PIPELINE, p.min_range, p.max_range, p.voxel)))
{
}

mrpt::viz::CPointCloudColoured::Ptr KeyframeMapVisualization::build_cloud(
  const mrpt::obs::CSensoryFrame & observations, const mrpt::poses::CPose3D & pose_in_map) const
{
  mrpt::maps::CSimpleMap sm;
  sm.insert(
    mrpt::poses::CPose3DPDFGaussian::Create(mrpt::poses::CPose3D()),
    std::make_shared<mrpt::obs::CSensoryFrame>(observations));

  mp2p_icp_filters::sm2mm_options_t opts;
  opts.verbosity = mrpt::system::LVL_ERROR;
  mp2p_icp::metric_map_t mm;
  mp2p_icp_filters::simplemap_to_metricmap(sm, mm, pipeline_, opts);
  if (mm.layers.count("points") == 0) {
    return {};
  }
  const auto pts = mm.point_layer("points");
  if (!pts || pts->empty()) {
    return {};
  }

  // Height in the map frame relative to the keyframe: the z row of its rotation.
  const auto R = pose_in_map.getRotationMatrix();
  const double span = std::max(params_.color_max - params_.color_min, 1e-3);
  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();

  auto cloud = mrpt::viz::CPointCloudColoured::Create();
  cloud->resize(xs.size());
  for (std::size_t i = 0; i < xs.size(); i++) {
    const double h = R(2, 0) * xs[i] + R(2, 1) * ys[i] + R(2, 2) * zs[i];
    const auto c = turbo_color((h - params_.color_min) / span);
    cloud->setPoint_fast(i, xs[i], ys[i], zs[i]);
    cloud->setPointColor_u8_fast(i, c.R, c.G, c.B);
  }
  cloud->setPointSize(params_.point_size);
  return cloud;
}

std::size_t KeyframeMapVisualization::add_keyframes(
  const std::vector<KeyframeData> & kfs, const std::vector<mrpt::poses::CPose3D> & poses_in_map,
  double max_seconds)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::size_t added = 0;
  for (const auto & kf : kfs) {
    if (kf.index < num_considered_) {
      continue;
    }
    if (
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > max_seconds) {
      break;
    }
    num_considered_ = kf.index + 1;
    if (kf.index >= poses_in_map.size()) {
      continue;
    }
    const auto & pose = poses_in_map[kf.index];
    const bool tooClose = std::any_of(clouds_.begin(), clouds_.end(), [&](const auto & c) {
      return c.index < poses_in_map.size() &&
             (poses_in_map[c.index].translation() - pose.translation()).norm() <
               params_.min_spacing;
    });
    if (tooClose) {
      continue;
    }
    auto cloud = build_cloud(kf.observations, pose);
    if (!cloud) {
      continue;
    }
    num_points_ += cloud->size();
    clouds_.push_back({kf.index, std::move(cloud)});
    added++;
  }
  return added;
}

mrpt::viz::CSetOfObjects::Ptr KeyframeMapVisualization::build_scene(
  const std::vector<mrpt::poses::CPose3D> & poses_in_map) const
{
  auto scene = mrpt::viz::CSetOfObjects::Create();
  for (const auto & c : clouds_) {
    if (c.index >= poses_in_map.size()) {
      continue;
    }
    auto glKf = mrpt::viz::CSetOfObjects::Create();
    glKf->setPose(poses_in_map[c.index]);
    glKf->insert(c.cloud);
    scene->insert(glKf);
  }
  return scene;
}

std::vector<mrpt::viz::CPointCloudColoured::Ptr> KeyframeMapVisualization::clouds() const
{
  std::vector<mrpt::viz::CPointCloudColoured::Ptr> out;
  out.reserve(clouds_.size());
  for (const auto & c : clouds_) {
    out.push_back(c.cloud);
  }
  return out;
}

}  // namespace mola::mapper
