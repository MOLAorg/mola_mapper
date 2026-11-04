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
 * @file   test-local-map-corrections.cpp
 * @brief  The mapper as a mola::LocalMapCorrectionSource, on a simulated
 *         warehouse run that revisits its start, with a drifting odometry.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/wrap2pi.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
namespace
{
using mrpt::poses::CPose3D;

constexpr double YAW_DRIFT_DEG = 0.2;  // per keyframe
constexpr double RADIUS = 15.0;        // [m]

std::shared_ptr<mola::mapper::Mapper> make_mapper(bool corrections)
{
  // The production detector pipeline, with candidates closer in time for this
  // short run, and no deskew (the simulated clouds have no point stamps):
  ::setenv("MIN_FRAMES_BETWEEN_LC", "20", 1);
  ::setenv("MOLA_DESKEW_METHOD", "MotionCompensationMethod::None", 1);
  auto m = std::make_shared<mola::mapper::Mapper>();
  m->setMinLoggingLevel(mrpt::system::LVL_WARN);
  const std::string yaml = std::string(R"###(
params:
  reference_frame_name: "map"
  enable_optimizer_thread: false
  gnss_sensor_labels_re: ""
  loop_closure_enabled: true
  enable_loop_closure_thread: false
  loop_closure_finalize_rounds: 0
  local_map_corrections: )###") +
                           (corrections ? "true" : "false") + R"###(
  loop_closure_pipeline_file: ")###" +
                           MOLA_MAPPER_LC_PIPELINE + "\"\n";
  m->initialize(mrpt::containers::yaml::FromText(yaml));
  return m;
}

double distance(const CPose3D & a, const CPose3D & b)
{
  return (a.translation() - b.translation()).norm();
}

void test_corrections_after_loop_closures(bool corrections)
{
  mrpt::maps::CSimpleMap sm;
  ASSERT_(sm.loadFromFile(MOLA_TEST_WAREHOUSE_SIMPLEMAP));
  ASSERT_GT_(sm.size(), 10U);

  std::vector<CPose3D> gt;
  for (const auto & kf : sm) {
    gt.push_back(kf.pose->getMeanVal());
  }

  auto m = make_mapper(corrections);
  std::vector<mola::LocalMapCorrection> received;
  m->subscribe_to_local_map_corrections(
    [&](const mola::LocalMapCorrection & c) { received.push_back(c); });

  // The run with an odometry that drifts in heading:
  std::vector<CPose3D> odoPoses;
  CPose3D odo = gt.front();
  for (std::size_t i = 0; i < sm.size(); i++) {
    if (i > 0) {
      odo = odo + (gt[i] - gt[i - 1]) + CPose3D(0, 0, 0, mrpt::DEG2RAD(YAW_DRIFT_DEG), 0, 0);
    }
    const auto & sf = sm.get(i).sf;
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = sf->getObservationByIndex(0)->timestamp;
    req.source_frame_id = "lidar_odom_kf";
    req.pose_in_source.mean = odo;
    req.pose_in_source.cov.setDiagonal(1e-6);
    req.observations = *sf;
    odoPoses.push_back(odo);
    const auto id = m->requestInsertKeyframe(req);
    ASSERT_(id.has_value());
    ASSERT_EQUAL_(*id, i);
  }
  const std::size_t newest = sm.size() - 1;

  // Without loop closures, no correction is advertised:
  m->optimize_now();
  ASSERT_(received.empty());

  const std::size_t loops = m->run_loop_closure_scan_now(true);
  std::cout << "Loop closures: " << loops << "\n";
  ASSERT_GT_(loops, 0U);

  m->optimize_now();
  if (!corrections) {
    ASSERT_(received.empty());
    return;
  }
  ASSERT_EQUAL_(received.size(), 1U);
  ASSERT_EQUAL_(received.front().new_loop_closures, loops);
  ASSERT_(received.front().timestamp == sm.get(newest).sf->getObservationByIndex(0)->timestamp);

  // A solve without new loops advertises nothing:
  m->optimize_now();
  ASSERT_EQUAL_(received.size(), 1U);

  const auto after = m->local_map_keyframes(newest, RADIUS);
  ASSERT_(after.has_value());
  const auto poses = m->keyframe_poses();
  // Keyframes of the start of the run, revisited at its end:
  std::size_t numEarlyVisit = 0;
  double errOdometry = 0;
  double errCorrected = 0;
  for (const auto & kf : *after) {
    ASSERT_LT_(kf.id, poses.size());
    // Poses relative to the reference, from the current estimates:
    const CPose3D expected = poses[kf.id] - poses[newest];
    ASSERT_LT_(distance(kf.pose_from_reference, expected), 1e-9);
    ASSERT_LE_(kf.pose_from_reference.translation().norm(), RADIUS + 1e-9);
    ASSERT_EQUAL_(kf.source_frame_id, std::string("lidar_odom_kf"));
    ASSERT_EQUAL_(kf.observations.size(), sm.get(kf.id).sf->size());
    if (newest - kf.id > sm.size() / 2) {
      const CPose3D gtRel = gt[kf.id] - gt[newest];
      numEarlyVisit++;
      errOdometry += distance(odoPoses[kf.id] - odoPoses[newest], gtRel);
      errCorrected += distance(kf.pose_from_reference, gtRel);
    }
  }
  ASSERT_GT_(numEarlyVisit, 0U);
  errOdometry /= static_cast<double>(numEarlyVisit);
  errCorrected /= static_cast<double>(numEarlyVisit);
  std::cout << "Keyframes within " << RADIUS << " m of the newest: " << after->size() << ", "
            << numEarlyVisit << " from the start of the run, relative position error "
            << errOdometry << " m with the odometry, " << errCorrected << " m corrected\n";

  // Oldest first, the reference included:
  for (std::size_t i = 1; i < after->size(); i++) {
    ASSERT_LT_(after->at(i - 1).id, after->at(i).id);
  }
  ASSERT_EQUAL_(after->back().id, newest);

  // The corrected poses of the earlier visit are much better than the odometry:
  ASSERT_LT_(errCorrected, 0.3 * errOdometry);

  // An unknown reference:
  ASSERT_(!m->local_map_keyframes(sm.size(), RADIUS).has_value());
}

}  // namespace
#endif

int main()
{
  try {
#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
    test_corrections_after_loop_closures(true);
    test_corrections_after_loop_closures(false);  // disabled: none advertised
    std::cout << "All tests passed.\n";
#else
    std::cout << "mola_kernel without LocalMapCorrectionSource: skipped.\n";
#endif
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
