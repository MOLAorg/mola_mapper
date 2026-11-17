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
 * @file   test-keyframe-map-visualization.cpp
 * @brief  Unit test: the GUI map of keyframe clouds.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/KeyframeMapVisualization.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <iostream>
#include <vector>

namespace
{
using mola::mapper::KeyframeMapParams;
using mola::mapper::KeyframeMapVisualization;
using mrpt::poses::CPose3D;

/// A scan with one point at each given position (sensor at the vehicle origin).
mrpt::obs::CSensoryFrame make_scan(const std::vector<mrpt::math::TPoint3D> & pts)
{
  auto pc = mrpt::maps::CSimplePointsMap::Create();
  for (const auto & p : pts) {
    pc->insertPoint(p);
  }
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->pointcloud = pc;
  mrpt::obs::CSensoryFrame sf;
  sf.insert(obs);
  return sf;
}

/// Keyframes 1 m apart along x, each one seeing a point 5 m ahead.
std::vector<KeyframeMapVisualization::KeyframeData> make_keyframes(
  std::size_t n, std::vector<CPose3D> & poses)
{
  std::vector<KeyframeMapVisualization::KeyframeData> kfs;
  for (std::size_t i = 0; i < n; i++) {
    poses.emplace_back(static_cast<double>(i), 0, 0, 0, 0, 0);
    kfs.push_back({i, make_scan({{5.0, 0, 0}})});
  }
  return kfs;
}

void test_spacing_and_poses()
{
  KeyframeMapParams p;
  p.min_spacing = 2.5;
  KeyframeMapVisualization m(p);
  std::vector<CPose3D> poses;
  const auto kfs = make_keyframes(10, poses);

  // No time: nothing is considered yet.
  ASSERT_EQUAL_(m.add_keyframes(kfs, poses, -1.0), 0U);
  ASSERT_EQUAL_(m.num_considered(), 0U);

  // Keyframes 0, 3, 6, 9 contribute (2.5 m spacing, 1 m apart):
  ASSERT_EQUAL_(m.add_keyframes(kfs, poses), 4U);
  ASSERT_EQUAL_(m.num_considered(), 10U);
  ASSERT_EQUAL_(m.num_clouds(), 4U);
  ASSERT_EQUAL_(m.num_points(), 4U);
  // Already considered keyframes are skipped:
  ASSERT_EQUAL_(m.add_keyframes(kfs, poses), 0U);

  // One node per cloud, at its keyframe pose, with the cloud in the keyframe frame:
  const auto scene = m.scene();
  ASSERT_EQUAL_(scene->size(), 4U);
  const auto glKf = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(*std::next(scene->begin()));
  ASSERT_(glKf);
  ASSERT_NEAR_((CPose3D(glKf->getPose()).asVectorVal() - poses[3].asVectorVal()).norm(), 0.0, 1e-6);
  const auto cloud = std::dynamic_pointer_cast<mrpt::viz::CPointCloudColoured>(*glKf->begin());
  ASSERT_(cloud);
  ASSERT_EQUAL_(cloud->size(), 1U);
  ASSERT_NEAR_(cloud->getPoint3Df(0).x, 5.0f, 1e-3f);

  // Nothing moved yet:
  ASSERT_(m.pose_changes(poses).empty());
  // A keyframe moves: only its node, once.
  poses[3] = CPose3D(3.0, 1.0, 0.5, 0.1, 0, 0);
  poses[4] = CPose3D(4.0, 2.0, 0.0, 0, 0, 0);  // not in the map
  const auto moved = m.pose_changes(poses);
  ASSERT_EQUAL_(moved.size(), 1U);
  ASSERT_(moved.front().node == glKf);
  ASSERT_NEAR_((moved.front().pose.asVectorVal() - poses[3].asVectorVal()).norm(), 0.0, 1e-9);
  ASSERT_(m.pose_changes(poses).empty());

  // The same nodes in every scene (the visualizer keeps their GPU buffers):
  const auto scene2 = m.scene();
  ASSERT_(*std::next(scene2->begin()) == glKf);
}

void test_glides()
{
  using mola::mapper::NodeAnimator;
  auto node = mrpt::viz::CSetOfObjects::Create();
  const CPose3D a(0, 0, 0, 0, 0, 0);
  const CPose3D b(10, 0, 0, M_PI / 2, 0, 0);
  NodeAnimator anim;
  anim.add({{node, b, a}}, 1.0 /*start*/, 2.0 /*duration*/, 0.0 /*now*/);

  // Not started yet: nothing to set.
  ASSERT_(anim.step(0.5).empty());
  // Half way (cosine easing: half the motion), translation and rotation:
  auto p = anim.step(2.0);
  ASSERT_EQUAL_(p.size(), 1U);
  ASSERT_NEAR_(p[0].pose.x(), 5.0, 1e-6);
  ASSERT_NEAR_(p[0].pose.yaw(), M_PI / 4, 1e-6);
  // A new target meanwhile: on from where it is, without a jump.
  const CPose3D c(10, 10, 0, M_PI / 2, 0, 0);
  anim.add({{node, c, b}}, 2.0, 2.0, 2.0);
  p = anim.step(2.0);
  ASSERT_NEAR_(p[0].pose.x(), 5.0, 1e-6);
  // Done: the final pose once, then nothing.
  p = anim.step(10.0);
  ASSERT_EQUAL_(p.size(), 1U);
  ASSERT_NEAR_((p[0].pose.asVectorVal() - c.asVectorVal()).norm(), 0.0, 1e-9);
  ASSERT_(anim.step(11.0).empty());
  ASSERT_(anim.empty());

  // A glide waiting to start keeps waiting if its node gets a new target:
  anim.add({{node, b, a}}, 20.0, 2.0, 12.0);
  anim.add({{node, c, b}}, 13.0, 2.0, 13.0);
  ASSERT_(anim.step(19.0).empty());
  p = anim.step(22.0);
  ASSERT_NEAR_((p[0].pose.asVectorVal() - c.asVectorVal()).norm(), 0.0, 1e-9);
}

void test_glides_many_and_small()
{
  using mola::mapper::NodeAnimator;
  NodeAnimator anim;
  // Many nodes finishing in the same tick:
  std::vector<mola::mapper::KeyframeMapVisualization::NodePose> changes;
  for (int i = 0; i < 5000; i++) {
    changes.push_back(
      {mrpt::viz::CSetOfObjects::Create(), CPose3D(i, 1, 0, 0, 0, 0), CPose3D(i, 0, 0, 0, 0, 0)});
  }
  anim.add(changes, 0.0, 1.0, 0.0);
  ASSERT_EQUAL_(anim.size(), 5000U);
  ASSERT_EQUAL_(anim.step(0.5).size(), 5000U);
  ASSERT_EQUAL_(anim.step(1.0).size(), 5000U);
  ASSERT_(anim.empty());

  // Tiny changes (every solve) are applied at once:
  auto node = mrpt::viz::CSetOfObjects::Create();
  anim.add({{node, CPose3D(0.005, 0, 0, 0, 0, 0), CPose3D()}}, 5.0, 2.0, 1.0);
  auto p = anim.step(1.0);
  ASSERT_EQUAL_(p.size(), 1U);
  ASSERT_NEAR_(p[0].pose.x(), 0.005, 1e-9);
  ASSERT_(anim.empty());

  // A small retarget while gliding keeps the glide (no restart of the easing):
  anim.add({{node, CPose3D(10, 0, 0, 0, 0, 0), CPose3D()}}, 0.0, 2.0, 0.0);
  ASSERT_NEAR_(anim.step(1.0)[0].pose.x(), 5.0, 1e-6);
  anim.add({{node, CPose3D(10.2, 0, 0, 0, 0, 0), CPose3D(10, 0, 0, 0, 0, 0)}}, 1.0, 2.0, 1.0);
  ASSERT_NEAR_(anim.step(1.0)[0].pose.x(), 5.1, 1e-6);  // half of the new motion, same timing
  ASSERT_NEAR_(anim.step(2.0)[0].pose.x(), 10.2, 1e-9);
  ASSERT_(anim.empty());

  // Large rotations: the short way around (170 deg -> -170 deg via 180):
  anim.add(
    {{node, CPose3D(0, 0, 0, mrpt::DEG2RAD(-170.0), 0, 0),
      CPose3D(0, 0, 0, mrpt::DEG2RAD(170.0), 0, 0)}},
    0.0, 2.0, 0.0);
  ASSERT_NEAR_(std::abs(anim.step(1.0)[0].pose.yaw()), M_PI, 1e-6);
}

void test_correction_region()
{
  using mola::mapper::correction_region;
  std::vector<mola::mapper::KeyframeMapVisualization::NodePose> changes;
  const auto add = [&](double x, double y, double shift) {
    changes.push_back(
      {mrpt::viz::CSetOfObjects::Create(), CPose3D(x + shift, y, 0, 0, 0, 0),
       CPose3D(x, y, 0, 0, 0, 0)});
  };
  add(0, 0, 2.0);    // the largest
  add(10, 0, 1.5);   // near, large enough
  add(20, 0, 0.5);   // near, too small
  add(500, 0, 1.8);  // large, but far away
  double maxShift = 0;
  const auto box = correction_region(changes, 0.5, 80.0, &maxShift);
  ASSERT_(box);
  ASSERT_NEAR_(maxShift, 2.0, 1e-9);
  ASSERT_NEAR_(box->min.x, 0.0, 1e-9);  // old position included
  ASSERT_NEAR_(box->max.x, 11.5, 1e-9);
  ASSERT_(!correction_region(changes, 3.0, 80.0));
}

void test_range_filter()
{
  KeyframeMapParams p;
  p.min_range = 1.5;
  p.max_range = 50.0;
  const KeyframeMapVisualization m(p);
  const auto c = m.build_cloud(make_scan({{1.0, 0, 0}, {10.0, 0, 0}, {60.0, 0, 0}}), CPose3D());
  ASSERT_(c);
  ASSERT_EQUAL_(c->size(), 1U);
  // Nothing in range: no cloud.
  ASSERT_(!m.build_cloud(make_scan({{0.5, 0, 0}}), CPose3D()));
}

void test_colors()
{
  KeyframeMapParams p;
  p.color_min = -2.0;
  p.color_max = 12.0;
  const KeyframeMapVisualization m(p);
  const auto expected = mola::mapper::turbo_color((5.0 - p.color_min) / 14.0);

  const auto check = [&](const mrpt::viz::CPointCloudColoured::Ptr & c) {
    ASSERT_(c);
    ASSERT_EQUAL_(c->size(), 1U);
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    c->getPointColor_fast(0, r, g, b);
    ASSERT_EQUAL_(r, expected.R);
    ASSERT_EQUAL_(g, expected.G);
    ASSERT_EQUAL_(b, expected.B);
  };
  // 5 m above a level keyframe, at any position in the map:
  check(m.build_cloud(make_scan({{0, 0, 5.0}}), CPose3D(10, 20, 30, 0.5, 0, 0)));
  // 5 m ahead of a keyframe pitched up 90 deg is also 5 m above it:
  check(m.build_cloud(make_scan({{5.0, 0, 0}}), CPose3D::FromYawPitchRoll(0, -M_PI / 2, 0)));

  // The colormap ends:
  ASSERT_(!(mola::mapper::turbo_color(0.0) == mola::mapper::turbo_color(1.0)));
  ASSERT_(mola::mapper::turbo_color(-1.0) == mola::mapper::turbo_color(0.0));
}

}  // namespace

int main()
{
  try {
    test_spacing_and_poses();
    test_glides();
    test_glides_many_and_small();
    test_correction_region();
    test_range_filter();
    test_colors();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
