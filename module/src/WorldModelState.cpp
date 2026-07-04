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
 * @file   WorldModelState.cpp
 * @brief  Central, optimized world model implementation.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/WorldModelState.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include "GtsamData.h"

namespace mola::mapper
{

WorldModelState::WorldModelState() : gtsam(mrpt::make_impl<GtsamData>()) {}

WorldModelState::~WorldModelState() = default;

KeyFrameID WorldModelState::last_kf_id() const
{
  ASSERT_(!time_to_kf_id.empty());
  return time_to_kf_id.getInverseMap().rbegin()->first;
}

KeyFrameID WorldModelState::generate_new_kf_id() const
{
  return time_to_kf_id.empty() ? 0 : last_kf_id() + 1;
}

void WorldModelState::add_kf_connectivity(KeyFrameID id1, KeyFrameID id2)
{
  if (id1 == id2) {
    return;
  }
  kf_connectivity.insertEdge(id1, id2, DummyEdgeData());
  kf_connectivity.insertEdge(id2, id1, DummyEdgeData());
}

std::set<KeyFrameID> WorldModelState::get_keyframes_in_topological_radius(
  KeyFrameID id, size_t max_topological_distance) const
{
  // Simple breadth-first expansion over the (symmetric) connectivity edges.
  std::set<KeyFrameID> result;
  result.insert(id);

  if (max_topological_distance == 0) {
    return result;
  }

  std::set<KeyFrameID> frontier = {id};
  for (size_t depth = 0; depth < max_topological_distance && !frontier.empty(); ++depth) {
    std::set<KeyFrameID> nextFrontier;
    for (const auto & [edgeNodes, edgeData] : kf_connectivity.edges) {
      (void)edgeData;
      const KeyFrameID a = edgeNodes.first;
      const KeyFrameID b = edgeNodes.second;
      if (frontier.count(a) != 0U && result.insert(b).second) {
        nextFrontier.insert(b);
      }
    }
    frontier.swap(nextFrontier);
  }
  return result;
}

mrpt::maps::CSimpleMap WorldModelState::as_simple_map(std::vector<KeyFrameID> * out_frame_ids) const
{
  mrpt::maps::CSimpleMap sm;
  if (out_frame_ids != nullptr) {
    out_frame_ids->clear();
  }
  for (const auto & [stamp, kfId] : time_to_kf_id) {
    const auto itObs = keyframe_observations.find(kfId);
    if (itObs == keyframe_observations.end()) {
      continue;
    }
    const auto itPose = last_estimated_states.find(kfId);
    const mrpt::poses::CPose3D pose = (itPose != last_estimated_states.end())
                                        ? itPose->second.pose
                                        : mrpt::poses::CPose3D::Identity();

    auto posePdf = mrpt::poses::CPose3DPDFGaussian::Create(pose);
    auto sf = std::make_shared<mrpt::obs::CSensoryFrame>(itObs->second);
    sm.insert(posePdf, sf);
    if (out_frame_ids != nullptr) {
      out_frame_ids->push_back(kfId);
    }
  }
  return sm;
}

void WorldModelState::clear()
{
  time_to_kf_id.clear();
  keyframe_observations.clear();
  last_estimated_states.clear();
  last_estimated_frames.clear();
  last_raw_pose_by_source.clear();
  kf_connectivity.clearEdges();
  known_odom_frames.clear();
  next_odom_frame_id = 1;
  geo_reference.reset();
  tentative_geo_coord_reference.reset();

  gtsam->isam2.reset();
  gtsam->newFactors.resize(0);
  gtsam->newValues.clear();
  gtsam->estimate.clear();
}

}  // namespace mola::mapper
