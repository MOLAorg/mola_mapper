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
#include <mola_mapper/Mapper.h>
#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace
{
using mola::mapper::LoopConsistencyGate;
using mola::mapper::LoopConsistencyParams;
using mola::mapper::LoopForConsistency;
using mrpt::poses::CPose3D;

constexpr std::size_t LAP = 300;  // keyframes (1 m apart) of the 100 x 50 m rectangle

/** The rectangle driven twice (the second time the other way around if
 *  `reverse`), and an odometry with a scale error per lap and a heading error at
 *  each corner per lap. */
struct Run
{
  std::vector<CPose3D> gt;
  std::vector<CPose3D> odo;
  std::vector<double> path;
};

Run make_run(
  double scale1, double cornerDeg1, double scale2, double cornerDeg2, bool reverse = false)
{
  Run r;
  CPose3D gt;
  for (const int side : {100, 50, 100, 50}) {
    for (int i = 0; i < side; i++) {
      r.gt.push_back(gt);
      gt = gt + CPose3D(1.0, 0, 0, i == side - 1 ? M_PI / 2 : 0.0, 0, 0);
    }
  }
  for (std::size_t k = 0; k < LAP; k++) {
    r.gt.push_back(reverse ? r.gt[LAP - 1 - k] + CPose3D(0, 0, 0, M_PI, 0, 0) : r.gt[k]);
  }
  CPose3D odo;
  double path = 0;
  for (std::size_t k = 0; k < r.gt.size(); k++) {
    r.odo.push_back(odo);
    r.path.push_back(path);
    if (k + 1 == r.gt.size()) {
      break;
    }
    const bool second = k + 1 >= LAP;
    const double scale = second ? scale2 : scale1;
    const double corner = second ? cornerDeg2 : cornerDeg1;
    const CPose3D inc = r.gt[k + 1] - r.gt[k];
    const bool turn = std::abs(inc.yaw()) > 1.0;
    const CPose3D step(
      inc.x() * (1 + scale), inc.y() * (1 + scale), 0,
      inc.yaw() + (turn ? mrpt::DEG2RAD(corner) : 0.0), 0, 0);
    odo = odo + step;
    path += step.translation().norm();
  }
  return r;
}

/// The keyframe of the second lap at the same place as keyframe i of the first.
std::size_t revisit(std::size_t i, bool reverse) { return reverse ? 2 * LAP - 1 - i : i + LAP; }

LoopForConsistency make_loop(
  const Run & r, std::size_t a, std::size_t b, const CPose3D & error = CPose3D())
{
  LoopForConsistency l;
  l.from = a;
  l.to = b;
  l.relative_pose = (r.gt[b] - r.gt[a]) + error;
  l.from_odometry = r.odo[a];
  l.to_odometry = r.odo[b];
  l.from_path = r.path[a];
  l.to_path = r.path[b];
  return l;
}

/// A correct loop, with a small registration error.
LoopForConsistency correct(const Run & r, std::size_t i, bool reverse = false)
{
  return make_loop(r, i, revisit(i, reverse), CPose3D(0.05, -0.03, 0.01, 0.002, 0, 0));
}

/// A place matched to the wrong one, `offset` meters along the street: all of
/// them agree with each other, not with the correct ones.
LoopForConsistency aliased(const Run & r, std::size_t i, double offset = 4.5)
{
  return make_loop(r, i, revisit(i, false), CPose3D(offset, 0, 0, 0, 0, 0));
}

std::set<std::size_t> accepted_from(const LoopConsistencyGate & g)
{
  std::set<std::size_t> out;
  for (std::size_t k = 0; k < g.size(); k++) {
    if (g.is_accepted(k)) {
      out.insert(std::min(g.loop(k).from, g.loop(k).to));
    }
  }
  return out;
}

void test_correct_loops_kept()
{
  const LoopConsistencyParams p;
  for (const bool reverse : {false, true}) {
    // Odometry errors in both laps, different in each:
    const Run r = make_run(0.01, 0.3, 0.03, 0.5, reverse);
    LoopConsistencyGate g(p);
    std::size_t n = 0;
    for (std::size_t i = 2; i < LAP - 2; i += 4) {
      ASSERT_(g.add({correct(r, i, reverse)}).empty());
      n++;
    }
    std::cout << "correct loops (" << (reverse ? "reversed" : "same direction")
              << " revisit): " << n << ", rejected " << g.rejected_count() << "\n";
    ASSERT_EQUAL_(g.rejected_count(), 0U);
  }
}

void test_aliased_loop_rejected()
{
  const LoopConsistencyParams p;
  const Run r = make_run(0.01, 0.3, 0.02, 0.3);
  for (const bool swapped : {false, true}) {
    LoopConsistencyGate g(p);
    ASSERT_(g.add({correct(r, 20), correct(r, 30), correct(r, 40)}).empty());
    // One aliased loop among them, stated either way, and one more correct one:
    auto a = aliased(r, 35, 3.0);
    if (swapped) {
      std::swap(a.from, a.to);
      std::swap(a.from_odometry, a.to_odometry);
      std::swap(a.from_path, a.to_path);
      a.relative_pose = CPose3D() - a.relative_pose;
    }
    const auto changed = g.add({a, correct(r, 45)});
    ASSERT_EQUAL_(changed.size(), 1U);
    ASSERT_EQUAL_(changed.at(0), 3U);
    ASSERT_(!g.is_accepted(3));
    ASSERT_(g.is_accepted(4));
  }
}

void test_single_neighbor_does_not_decide()
{
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  // One aliased and one correct loop: one inconsistent neighbor each, both kept.
  ASSERT_(g.add({aliased(r, 35), correct(r, 30)}).empty());
  ASSERT_EQUAL_(g.rejected_count(), 0U);
}

void test_tie_is_not_an_outlier()
{
  // X is consistent with C1, C2 and inconsistent with B1, B2 (2 of 4); the B
  // ones are consistent with everything but X. A tie keeps X.
  LoopConsistencyParams p;
  p.max_xyz = 1.0;
  p.max_xyz_per_m = 0.02;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  const CPose3D plus(1.0, 0, 0, 0, 0, 0);
  const CPose3D minus(-1.0, 0, 0, 0, 0, 0);
  ASSERT_(
    g.add({make_loop(r, 20, revisit(20, false)), make_loop(r, 25, revisit(25, false)),
           make_loop(r, 30, revisit(30, false), plus), make_loop(r, 35, revisit(35, false), plus),
           make_loop(r, 40, revisit(40, false), minus)})
      .empty());
}

void test_aliased_first_then_correct_majority()
{
  // Two aliased loops first (they agree with each other), then correct ones one
  // at a time: once the correct ones are the majority, the aliased ones are
  // dropped and every correct one is accepted.
  const LoopConsistencyParams p;
  const Run r = make_run(0.01, 0.3, 0.02, 0.3);
  LoopConsistencyGate g(p);
  ASSERT_(g.add({aliased(r, 20), aliased(r, 30)}).empty());
  std::size_t n = 0;
  for (std::size_t i = 22; i < 60; i += 3) {
    (void)g.add({correct(r, i)});
    n++;
  }
  ASSERT_(!g.is_accepted(0));
  ASSERT_(!g.is_accepted(1));
  ASSERT_EQUAL_(g.rejected_count(), 2U);
  ASSERT_EQUAL_(accepted_from(g).size(), n);
}

void test_order_independent()
{
  // The same loops (correct, aliased by 3-5 m, and one far off), in different
  // orders and batches, give the same accepted set.
  const LoopConsistencyParams p;
  const Run r = make_run(0.01, 0.3, 0.02, 0.3);
  std::vector<LoopForConsistency> loops;
  for (std::size_t i = 20; i < 60; i += 4) {
    loops.push_back(correct(r, i));
  }
  for (const double off : {3.0, 4.0, 5.0}) {
    loops.push_back(aliased(r, 27 + 2 * static_cast<std::size_t>(off), off));  // 33, 35, 37
  }
  loops.push_back(aliased(r, 45, 12.0));
  // Errors with a rotation, some near the limits (where the frame the cycle
  // error is measured in matters):
  std::size_t i = 22;
  for (const auto & [yawDeg, lateral] :
       std::vector<std::pair<double, double>>{{0.5, 4.1}, {1.0, 3.0}, {2.0, 2.0}, {0.5, 1.2}}) {
    loops.push_back(
      make_loop(r, i, revisit(i, false), CPose3D(0, lateral, 0, mrpt::DEG2RAD(yawDeg), 0, 0)));
    i += i == 26 ? 16 : 4;
  }
  std::optional<std::set<std::size_t>> reference;
  std::mt19937 rng(3);
  for (int trial = 0; trial < 20; trial++) {
    auto order = loops;
    std::shuffle(order.begin(), order.end(), rng);
    LoopConsistencyGate g(p);
    for (std::size_t k = 0; k < order.size();) {
      const std::size_t batch = 1 + (k + trial) % 3;
      std::vector<LoopForConsistency> b(
        order.begin() + k, order.begin() + std::min(order.size(), k + batch));
      (void)g.add(b);
      k += b.size();
    }
    const auto acc = accepted_from(g);
    if (!reference) {
      reference = acc;
    }
    ASSERT_(acc == *reference);
  }
  for (std::size_t k = 20; k < 60; k += 4) {
    ASSERT_(reference->count(k) == 1);  // all the correct ones
  }
  for (const std::size_t k : {33U, 35U, 37U, 45U}) {
    ASSERT_(reference->count(k) == 0);  // none of the aliased ones
  }
}

void test_symmetric()
{
  // Two loops 40 m apart, one with a rotation and a lateral error: the verdict
  // does not depend on which one is first.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  const LoopConsistencyGate g(p);
  std::size_t numConsistent = 0;
  std::size_t total = 0;
  for (double deg = 0; deg <= 5.0; deg += 0.5) {
    for (double dy = 0; dy < 5.0; dy += 0.1) {
      const auto x = correct(r, 10);
      const auto y =
        make_loop(r, 50, revisit(50, false), CPose3D(0, dy, 0, mrpt::DEG2RAD(deg), 0, 0));
      const auto a = g.consistent(x, y);
      const auto b = g.consistent(y, x);
      ASSERT_(a.has_value() && b.has_value());
      ASSERT_EQUAL_(*a, *b);
      numConsistent += *a ? 1 : 0;
      total++;
    }
  }
  ASSERT_(numConsistent > 0 && numConsistent < total);  // both verdicts exercised

  // As the last loop or as the first one, the same verdict:
  for (const double dy : {4.0, 4.1, 4.2}) {
    const auto y =
      make_loop(r, 50, revisit(50, false), CPose3D(0, dy, 0, mrpt::DEG2RAD(0.5), 0, 0));
    LoopConsistencyGate last(p);
    (void)last.add({correct(r, 10)});
    (void)last.add({correct(r, 13)});
    (void)last.add({correct(r, 16)});
    (void)last.add({y});
    LoopConsistencyGate first(p);
    (void)first.add({y});
    (void)first.add({correct(r, 10)});
    (void)first.add({correct(r, 13)});
    (void)first.add({correct(r, 16)});
    ASSERT_EQUAL_(last.is_accepted(3), first.is_accepted(0));
  }
}

void test_stranded_loop_accepted_again()
{
  // A correct loop C outvoted by two aliased ones next to it, which are outvoted
  // in turn by three correct ones further on (not neighbors of C): C is left
  // without any inconsistent accepted neighbor, and is accepted again.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  (void)g.add(
    {correct(r, 2), aliased(r, 32), aliased(r, 37), correct(r, 62), correct(r, 67),
     correct(r, 72)});
  ASSERT_((accepted_from(g) == std::set<std::size_t>{2, 62, 67, 72}));
}

void test_two_wrong_then_more_correct()
{
  // One correct loop, then two aliased ones in one scan: two against one, the
  // correct one is rejected (no way to tell). Three more correct ones turn the
  // majority, and it is accepted again.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  ASSERT_(g.add({correct(r, 25)}).empty());
  const auto c1 = g.add({aliased(r, 20), aliased(r, 30)});
  ASSERT_EQUAL_(c1.size(), 1U);
  ASSERT_EQUAL_(c1.at(0), 0U);
  const auto c2 = g.add({correct(r, 27), correct(r, 33), correct(r, 36)});
  ASSERT_EQUAL_(c2.size(), 3U);  // 0 accepted again, 1 and 2 dropped
  ASSERT_(g.is_accepted(0));
  ASSERT_(!g.is_accepted(1));
  ASSERT_(!g.is_accepted(2));
}

void test_worst_outlier_first()
{
  // Three correct loops, two aliased ones and one far off, in one batch: the far
  // off one makes every loop an outlier at first; dropping the worst one first
  // leaves the correct ones.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  (void)g.add(
    {correct(r, 25), correct(r, 33), correct(r, 36), aliased(r, 20), aliased(r, 30),
     aliased(r, 28, 9.0)});
  ASSERT_((accepted_from(g) == std::set<std::size_t>{25, 33, 36}));
}

void test_exact_tie_drops_older_pair()
{
  // Two against two, all with the same votes: the oldest keyframe pair is
  // dropped first, which settles it.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  (void)g.add({aliased(r, 35), aliased(r, 30), correct(r, 25), correct(r, 20)});
  ASSERT_((accepted_from(g) == std::set<std::size_t>{30, 35}));
}

void test_far_at_one_end_not_compared()
{
  // A wrong loop whose first keyframe is next to those of the correct ones but
  // whose second one is 60 m further: not a neighbor, so not judged by them.
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  ASSERT_(g.add({correct(r, 20), correct(r, 25), correct(r, 30)}).empty());
  ASSERT_(g.add({make_loop(r, 25, revisit(25, false) + 60, CPose3D(5, 0, 0, 0, 0, 0))}).empty());
  // Far at both ends either:
  ASSERT_(g.add({make_loop(r, 200, revisit(200, false), CPose3D(5, 0, 0, 0, 0, 0))}).empty());
  ASSERT_EQUAL_(g.rejected_count(), 0U);
}

void test_other_chain_not_compared()
{
  const LoopConsistencyParams p;
  const Run r = make_run(0, 0, 0, 0);
  LoopConsistencyGate g(p);
  ASSERT_(g.add({correct(r, 20), correct(r, 25), correct(r, 30)}).empty());
  // The same wrong loop, but its second keyframe from another front end:
  auto w = aliased(r, 27);
  w.to_chain = 1;
  ASSERT_(g.add({w}).empty());
  // With both in chain 0, rejected:
  ASSERT_EQUAL_(g.add({aliased(r, 28)}).size(), 1U);
}

// ---------------------------------------------------------------------------
// The mapper: merge, drops and re-acceptance, bookkeeping, optimizer
// notification and local map corrections.
// ---------------------------------------------------------------------------
mola::ProposedLoopEdge edge(const LoopForConsistency & l)
{
  mola::ProposedLoopEdge e;
  e.from = l.from;
  e.to = l.to;
  e.relative_pose.mean = l.relative_pose;
  e.relative_pose.cov.setDiagonal(1e-4);
  return e;
}

void test_mapper()
{
  auto m = std::make_shared<mola::mapper::Mapper>();
  m->setMinLoggingLevel(mrpt::system::LVL_WARN);
  m->initialize(mrpt::containers::yaml::FromText(R"###(
params:
  reference_frame_name: "map"
  enable_optimizer_thread: false
  gnss_sensor_labels_re: ""
  loop_closure_enabled: false
  local_map_corrections: true
)###"));
#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
  std::vector<mola::LocalMapCorrection> received;
  m->subscribe_to_local_map_corrections(
    [&](const mola::LocalMapCorrection & c) { received.push_back(c); });
#endif
  const Run r = make_run(0.005, 0.2, 0.01, 0.2);
  for (std::size_t i = 0; i < r.gt.size(); i++) {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(1000.0 + 0.5 * static_cast<double>(i));
    req.source_frame_id = "odom_kf";
    req.pose_in_source.mean = r.odo[i];
    req.pose_in_source.cov.setDiagonal(1e-6);
    ASSERT_(m->requestInsertKeyframe(req).has_value());
  }
  m->optimize_now();
  ASSERT_(!m->solve_pending());

  using Pairs = std::vector<std::pair<std::size_t, std::size_t>>;
  const auto pairOf = [](std::size_t i) { return std::make_pair(i, revisit(i, false)); };
  const auto expectCorrection = [&](std::size_t n, std::size_t newLoops) {
#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
    ASSERT_EQUAL_(received.size(), n);
    if (n > 0) {
      ASSERT_EQUAL_(received.back().new_loop_closures, newLoops);
    }
#else
    (void)n;
    (void)newLoops;
#endif
  };

  // A correct loop:
  ASSERT_EQUAL_(m->add_loop_closures({edge(correct(r, 25))}), 1U);
  ASSERT_(m->solve_pending());
  m->optimize_now();
  expectCorrection(1, 1);

  // Two aliased loops (two against one): the correct one is dropped.
  ASSERT_EQUAL_(m->add_loop_closures({edge(aliased(r, 20)), edge(aliased(r, 30))}), 2U);
  ASSERT_((m->loop_closure_pairs() == Pairs{pairOf(20), pairOf(30)}));
  ASSERT_(m->solve_pending());
  m->optimize_now();
  expectCorrection(2, 2);

  // Two more correct ones: it is accepted again, the aliased ones dropped. As
  // many loops as before plus one, all of them new to the graph:
  ASSERT_EQUAL_(m->add_loop_closures({edge(correct(r, 33)), edge(correct(r, 36))}), 3U);
  ASSERT_((m->loop_closure_pairs() == Pairs{pairOf(25), pairOf(33), pairOf(36)}));
  ASSERT_(m->solve_pending());
  m->optimize_now();
  expectCorrection(3, 3);

  // A loop inconsistent with all of them: rejected, nothing changes, no solve
  // nor correction needed.
  ASSERT_EQUAL_(m->add_loop_closures({edge(aliased(r, 28, 9.0))}), 0U);
  ASSERT_(!m->solve_pending());
  m->optimize_now();
  expectCorrection(3, 3);  // none new

  // Pairs already proposed (accepted or not) are ignored:
  ASSERT_EQUAL_(m->add_loop_closures({edge(aliased(r, 20)), edge(correct(r, 25))}), 0U);
  ASSERT_EQUAL_(m->loop_closure_count(), 3U);

  // Three more aliased ones, the majority now: the correct ones are dropped, and
  // the end of the run solves the graph with the final loops:
  const auto before = m->keyframe_poses();
  ASSERT_EQUAL_(
    m->add_loop_closures({edge(aliased(r, 22)), edge(aliased(r, 34)), edge(aliased(r, 38))}), 5U);
  ASSERT_(m->solve_pending());
  m->finalize();
  ASSERT_(!m->solve_pending());
  const auto after = m->keyframe_poses();
  const std::size_t probe = revisit(30, false);
  ASSERT_GT_((after[probe].translation() - before[probe].translation()).norm(), 1.0);

  // Without the check, every loop goes in:
  auto m2 = std::make_shared<mola::mapper::Mapper>();
  m2->setMinLoggingLevel(mrpt::system::LVL_WARN);
  m2->initialize(mrpt::containers::yaml::FromText(R"###(
params:
  reference_frame_name: "map"
  enable_optimizer_thread: false
  gnss_sensor_labels_re: ""
  loop_closure_enabled: false
  loop_closure_consistency_check: false
)###"));
  for (std::size_t i = 0; i < r.gt.size(); i++) {
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = mrpt::Clock::fromDouble(1000.0 + 0.5 * static_cast<double>(i));
    req.source_frame_id = "odom_kf";
    req.pose_in_source.mean = r.odo[i];
    req.pose_in_source.cov.setDiagonal(1e-6);
    ASSERT_(m2->requestInsertKeyframe(req).has_value());
  }
  ASSERT_EQUAL_(
    m2->add_loop_closures(
      {edge(correct(r, 25)), edge(aliased(r, 20)), edge(aliased(r, 30)),
       edge(aliased(r, 28, 9.0))}),
    4U);
}

}  // namespace

int main()
{
  try {
    test_correct_loops_kept();
    test_aliased_loop_rejected();
    test_single_neighbor_does_not_decide();
    test_tie_is_not_an_outlier();
    test_aliased_first_then_correct_majority();
    test_order_independent();
    test_symmetric();
    test_stranded_loop_accepted_again();
    test_two_wrong_then_more_correct();
    test_worst_outlier_first();
    test_exact_tie_drops_older_pair();
    test_far_at_one_end_not_compared();
    test_other_chain_not_compared();
    test_mapper();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
