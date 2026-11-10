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
 * @file   test-loop-consistency.cpp
 * @brief  Unit test: loops inconsistent with their neighbor loops are rejected.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/LoopConsistency.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <iostream>
#include <vector>

namespace
{
using mola::mapper::find_inconsistent_loops;
using mola::mapper::LoopConsistencyParams;
using mola::mapper::LoopForConsistency;
using mola::mapper::OdometryChains;
using mrpt::poses::CPose3D;

/** A 100 x 50 m rectangle driven twice, a keyframe per meter. Ground truth, and
 *  odometry with 0.02 deg of heading drift and 1 % of scale error per meter. */
struct Run
{
  std::vector<CPose3D> gt;
  OdometryChains odo;
};

Run make_run()
{
  Run r;
  CPose3D gt;
  CPose3D odo;
  for (int lap = 0; lap < 2; lap++) {
    for (const int side : {100, 50, 100, 50}) {
      for (int i = 0; i < side; i++) {
        r.gt.push_back(gt);
        r.odo.pose_in_source.push_back(odo);
        r.odo.chain_id.push_back(0);
        const bool corner = (i == side - 1);
        const CPose3D step(1.0, 0, 0, corner ? M_PI / 2 : 0.0, 0, 0);
        gt = gt + step;
        odo = odo + CPose3D(1.01, 0, 0, step.yaw() + mrpt::DEG2RAD(0.02), 0, 0);
      }
    }
  }
  return r;
}

constexpr std::size_t LAP = 300;

/// A correct loop from keyframe i to the same place one lap later, with a small
/// registration error.
LoopForConsistency true_loop(const Run & r, std::size_t i, std::size_t shift = 0)
{
  LoopForConsistency l;
  l.from = i;
  l.to = i + LAP + shift;
  l.relative_pose = (r.gt[l.to] - r.gt[l.from]) + CPose3D(0.05, -0.03, 0, 0.002, 0, 0);
  return l;
}

/// A place matched to the wrong one, 20 m further along the same street: the
/// registration says "same place" while the scans are 20 m apart.
LoopForConsistency aliased_loop(const Run & r, std::size_t i)
{
  LoopForConsistency l;
  l.from = i;
  l.to = i + LAP + 20;
  l.relative_pose = r.gt[i + LAP] - r.gt[l.from];
  return l;
}

bool contains(const std::vector<std::size_t> & v, std::size_t k)
{
  return std::find(v.begin(), v.end(), k) != v.end();
}

void test_true_loops_kept()
{
  const auto r = make_run();
  const LoopConsistencyParams p;
  std::vector<LoopForConsistency> loops;
  for (std::size_t i = 10; i < 290; i += 7) {
    loops.push_back(true_loop(r, i, i % 3));
  }
  const auto bad = find_inconsistent_loops(r.odo, loops, 0, p);
  std::cout << "true loops: " << loops.size() << ", rejected " << bad.size() << "\n";
  ASSERT_(bad.empty());
}

void test_aliased_loop_rejected()
{
  const auto r = make_run();
  const LoopConsistencyParams p;
  // Accepted before: three correct loops along the first long side.
  std::vector<LoopForConsistency> loops = {true_loop(r, 20), true_loop(r, 30), true_loop(r, 40)};
  // New: one aliased loop among them, and one more correct one.
  loops.push_back(aliased_loop(r, 35));
  loops.push_back(true_loop(r, 45));
  const auto bad = find_inconsistent_loops(r.odo, loops, 3, p);
  ASSERT_EQUAL_(bad.size(), 1U);
  ASSERT_EQUAL_(bad.at(0), 3U);

  // The same, with the aliased loop stated the other way around (to -> from):
  auto rev = loops;
  std::swap(rev[3].from, rev[3].to);
  rev[3].relative_pose = CPose3D() - rev[3].relative_pose;
  const auto badRev = find_inconsistent_loops(r.odo, rev, 3, p);
  ASSERT_EQUAL_(badRev.size(), 1U);
  ASSERT_EQUAL_(badRev.at(0), 3U);
}

void test_single_neighbor_does_not_decide()
{
  const auto r = make_run();
  const LoopConsistencyParams p;
  // An aliased loop accepted first, then one correct loop: one inconsistent
  // neighbor each, so neither is dropped.
  std::vector<LoopForConsistency> loops = {aliased_loop(r, 35), true_loop(r, 30)};
  ASSERT_(find_inconsistent_loops(r.odo, loops, 1, p).empty());

  // Two more correct loops arrive: the early aliased one is dropped, all the
  // correct ones are kept.
  loops.push_back(true_loop(r, 40));
  loops.push_back(true_loop(r, 45));
  const auto bad = find_inconsistent_loops(r.odo, loops, 2, p);
  ASSERT_EQUAL_(bad.size(), 1U);
  ASSERT_EQUAL_(bad.at(0), 0U);
}

void test_far_loops_not_compared()
{
  const auto r = make_run();
  const LoopConsistencyParams p;
  // The aliased loop is far (along the path) from the correct ones: no evidence
  // against it, kept.
  std::vector<LoopForConsistency> loops = {
    true_loop(r, 20), true_loop(r, 25), true_loop(r, 30), aliased_loop(r, 200)};
  ASSERT_(find_inconsistent_loops(r.odo, loops, 3, p).empty());
}

void test_other_chain_not_compared()
{
  auto r = make_run();
  const LoopConsistencyParams p;
  // The second lap comes from another front end: its poses cannot be composed
  // with the first one, so loops between them are never compared.
  for (std::size_t i = LAP; i < r.odo.chain_id.size(); i++) {
    r.odo.chain_id[i] = 1;
  }
  std::vector<LoopForConsistency> loops = {
    true_loop(r, 20), true_loop(r, 30), true_loop(r, 40), aliased_loop(r, 35)};
  // Loops from chain 0 to chain 1 can still be compared with each other:
  // (from, from) and (to, to) are in the same chains.
  const auto bad = find_inconsistent_loops(r.odo, loops, 3, p);
  ASSERT_(contains(bad, 3U));

  // But not with a loop whose ends are both in chain 0:
  std::vector<LoopForConsistency> mixed = {true_loop(r, 20), true_loop(r, 30)};
  LoopForConsistency within;
  within.from = 10;
  within.to = 290;
  within.relative_pose = CPose3D(50, 0, 0, 0, 0, 0);  // wrong, but not comparable
  mixed.push_back(within);
  ASSERT_(find_inconsistent_loops(r.odo, mixed, 2, p).empty());
}

}  // namespace

int main()
{
  try {
    test_true_loops_kept();
    test_aliased_loop_rejected();
    test_single_neighbor_does_not_decide();
    test_far_loops_not_compared();
    test_other_chain_not_compared();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
