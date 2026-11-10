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
 * @file   test-graph-visualization.cpp
 * @brief  Unit test: the GUI scene of a keyframe graph.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GraphVisualization.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSphere.h>

#include <iostream>
#include <vector>

namespace
{
using mola::mapper::build_graph_visualization;
using mola::mapper::GraphVisualizationInput;
using mola::mapper::GraphVisualizationParams;

/// Two chains (10 + 5 keyframes), one loop within the first chain, one across
/// both, rejected candidates and a pair out of range.
GraphVisualizationInput make_input()
{
  GraphVisualizationInput in;
  for (int i = 0; i < 15; i++) {
    in.keyframes.emplace_back(i * 1.0, 0.5 * i, 0.0);
    in.chain.push_back(i < 10 ? 0 : 1);
  }
  in.loops = {{0, 9}, {2, 12}, {3, 99}};
  in.rejected_by_consistency = {{1, 8}};
  in.rejected_by_odometry = {{4, 7}, {5, 6}};
  return in;
}

struct Counts
{
  std::size_t spheres = 0;
  std::vector<mrpt::viz::CSetOfLines::Ptr> lines;
};

Counts count(mrpt::viz::CSetOfObjects & scene)
{
  Counts c;
  for (auto & o : scene) {
    if (auto s = std::dynamic_pointer_cast<mrpt::viz::CSphere>(o); s) {
      c.spheres++;
    } else if (auto l = std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(o); l) {
      c.lines.push_back(l);
    }
  }
  return c;
}

void test_defaults()
{
  const GraphVisualizationParams p;
  const auto scene = build_graph_visualization(make_input(), p);
  const auto c = count(*scene);
  ASSERT_EQUAL_(c.spheres, 15U);
  // Odometry edges and loops only: rejected loops are hidden by default.
  ASSERT_EQUAL_(c.lines.size(), 2U);
  // 9 + 4 consecutive pairs, none between the two chains:
  ASSERT_EQUAL_(c.lines[0]->size(), 13U);
  ASSERT_EQUAL_(c.lines[0]->getLineWidth(), p.odometry_edge_width);
  ASSERT_(c.lines[0]->getColor_u8() == p.odometry_edge_color);
  // The out of range pair is skipped; each loop is a bridge of three segments:
  ASSERT_EQUAL_(c.lines[1]->size(), 2U * 3U);
  ASSERT_EQUAL_(c.lines[1]->getLineWidth(), p.loop_edge_width);
  ASSERT_(c.lines[1]->getColor_u8() == p.loop_edge_color);
  ASSERT_(!(p.loop_edge_color == p.odometry_edge_color));
  // A loop edge joins the right keyframes:
  double x0 = 0;
  double y0 = 0;
  double z0 = 0;
  double x1 = 0;
  double y1 = 0;
  double z1 = 0;
  // The top of the second bridge, 2 -> 12:
  c.lines[1]->getLineByIndex(4, x0, y0, z0, x1, y1, z1);
  ASSERT_NEAR_(x0, 2.0, 1e-6);
  ASSERT_NEAR_(x1, 12.0, 1e-6);
  ASSERT_NEAR_(y1, 6.0, 1e-6);
  ASSERT_NEAR_(z0, p.loop_edge_lift, 1e-6);
  ASSERT_NEAR_(z1, p.loop_edge_lift, 1e-6);
  // And down to keyframe 12:
  c.lines[1]->getLineByIndex(5, x0, y0, z0, x1, y1, z1);
  ASSERT_NEAR_(x1, 12.0, 1e-6);
  ASSERT_NEAR_(z1, 0.0, 1e-6);
}

void test_options()
{
  GraphVisualizationParams p;
  p.show_rejected_loops = true;
  p.keyframe_sphere_radius = 0;
  p.odometry_edge_width = 0;
  p.loop_edge_lift = 0;
  const auto c = count(*build_graph_visualization(make_input(), p));
  ASSERT_EQUAL_(c.spheres, 0U);
  ASSERT_EQUAL_(c.lines.size(), 3U);
  ASSERT_EQUAL_(c.lines[1]->size(), 1U);
  ASSERT_(c.lines[1]->getColor_u8() == p.rejected_by_consistency_color);
  ASSERT_EQUAL_(c.lines[2]->size(), 2U);
  ASSERT_(c.lines[2]->getColor_u8() == p.rejected_by_odometry_color);
  ASSERT_EQUAL_(c.lines[2]->getLineWidth(), p.rejected_loop_edge_width);

  // An empty graph gives an empty, valid scene:
  const auto e = count(*build_graph_visualization(GraphVisualizationInput{}, p));
  ASSERT_EQUAL_(e.spheres, 0U);
  for (const auto & l : e.lines) {
    ASSERT_EQUAL_(l->size(), 0U);
  }
}

void test_params_from_yaml()
{
  GraphVisualizationParams p;
  p.loadFrom(mrpt::containers::yaml::FromText(
    "keyframe_sphere_radius: 0.4\nloop_edge_width: 6.0\nshow_rejected_loops: true\n"));
  ASSERT_NEAR_(p.keyframe_sphere_radius, 0.4f, 1e-6f);
  ASSERT_NEAR_(p.loop_edge_width, 6.0f, 1e-6f);
  ASSERT_(p.show_rejected_loops);
  ASSERT_NEAR_(p.odometry_edge_width, GraphVisualizationParams().odometry_edge_width, 1e-6f);
}

}  // namespace

int main()
{
  try {
    test_defaults();
    test_options();
    test_params_from_yaml();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
