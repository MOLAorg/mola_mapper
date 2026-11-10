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
 * @file   Mapper_GUI.cpp
 * @brief  The keyframe graph and the mapper status in a MOLA visualizer.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <optional>
#include <string>

namespace mola::mapper
{
struct Mapper::Gui
{
  // Written by the GUI thread (widgets), read by gui_update():
  std::mutex params_mtx;
  GraphVisualizationParams params;
  std::atomic_bool params_changed{true};

  mola::gui::LiveString::Ptr lbKeyframes = std::make_shared<mola::gui::LiveString>(" ");
  mola::gui::LiveString::Ptr lbLoops = std::make_shared<mola::gui::LiveString>(" ");
  mola::gui::LiveString::Ptr lbRejected = std::make_shared<mola::gui::LiveString>(" ");
  mola::gui::LiveString::Ptr lbFactors = std::make_shared<mola::gui::LiveString>(" ");
  mola::gui::LiveString::Ptr lbSolver = std::make_shared<mola::gui::LiveString>(" ");

  bool window_created = false;
  std::optional<std::chrono::steady_clock::time_point> last_update;

  // Front end chain of each keyframe and the newest keyframe of each chain,
  // extended as keyframes arrive (spinOnce() thread only):
  std::vector<std::size_t> kf_chain;
  std::map<std::string, std::size_t> chain_ids;
  std::vector<std::size_t> chain_newest_kf;

  /// Keyframes, solves, loop revision, rejected loops: the graph last drawn.
  using RedrawKey = std::array<std::size_t, 5>;
  std::optional<RedrawKey> last_drawn;
};

void Mapper::gui_initialize()
{
  visualizer_.reset();
  gui_.reset();
  const auto mods = findService<mola::VizInterface>();
  if (mods.empty()) {
    return;
  }
  visualizer_ = std::dynamic_pointer_cast<mola::VizInterface>(mods.front());
  if (!visualizer_) {
    return;
  }
  gui_ = std::make_shared<Gui>();
  gui_->params = params_.visualization;
  MRPT_LOG_INFO("Attached to a visualizer: the keyframe graph is shown in the GUI");
}

void Mapper::gui_build_window()
{
  using namespace mola::gui;
  auto g = gui_;

  WindowDescription desc;
  desc.title = "mola_mapper";
  desc.position = {5, 700};
  desc.size = {340, 0};

  {
    Tab tab;
    tab.title = "Status";
    tab.widgets.emplace_back(Label{g->lbKeyframes});
    tab.widgets.emplace_back(Label{g->lbLoops});
    tab.widgets.emplace_back(Label{g->lbRejected});
    tab.widgets.emplace_back(Label{g->lbFactors});
    tab.widgets.emplace_back(Label{g->lbSolver});
    if (lc_engine_ && lc_thread_running_.load()) {
      tab.widgets.emplace_back(Separator{});
      tab.widgets.emplace_back(
        Button{"Loop closure scan now", 0, "", 0, [this]() { request_loop_closure_scan(); }});
    }
    desc.tabs.emplace_back(std::move(tab));
  }

  {
    const auto p = g->params;
    // Every widget changes one parameter and asks for a redraw:
    const auto setter = [g](auto field) {
      return [g, field](auto v) {
        {
          auto lck = mrpt::lockHelper(g->params_mtx);
          g->params.*field = v;
        }
        g->params_changed = true;
      };
    };
    Tab tab;
    tab.title = "View";
    SliderFloat radius;
    radius.label = "Keyframe radius [m]";
    radius.initial_value = p.keyframe_sphere_radius;
    radius.max_value = 1.0f;
    radius.on_change = setter(&GraphVisualizationParams::keyframe_sphere_radius);
    tab.widgets.emplace_back(radius);

    SliderFloat odoWidth;
    odoWidth.label = "Odometry edges [px]";
    odoWidth.initial_value = p.odometry_edge_width;
    odoWidth.max_value = 10.0f;
    odoWidth.format_string = "%.1f";
    odoWidth.on_change = setter(&GraphVisualizationParams::odometry_edge_width);
    tab.widgets.emplace_back(odoWidth);

    SliderFloat loopWidth = odoWidth;
    loopWidth.label = "Loop closures [px]";
    loopWidth.initial_value = p.loop_edge_width;
    loopWidth.on_change = setter(&GraphVisualizationParams::loop_edge_width);
    tab.widgets.emplace_back(loopWidth);

    SliderFloat loopLift = radius;
    loopLift.label = "Loop bridge height [m]";
    loopLift.initial_value = p.loop_edge_lift;
    loopLift.max_value = 5.0f;
    loopLift.on_change = setter(&GraphVisualizationParams::loop_edge_lift);
    tab.widgets.emplace_back(loopLift);

    tab.widgets.emplace_back(CheckBox{
      "Show rejected loops (red: consistency, gray: odometry)", p.show_rejected_loops,
      setter(&GraphVisualizationParams::show_rejected_loops)});
    tab.widgets.emplace_back(CheckBox{
      "Align front end views with the map", p.align_front_end_frames,
      setter(&GraphVisualizationParams::align_front_end_frames)});
    desc.tabs.emplace_back(std::move(tab));
  }

  // Not waited for: the visualizer may never run it (e.g. its window closed).
  visualizer_->create_subwindow_from_description(desc);
}

void Mapper::gui_update()
{
  ASSERT_(visualizer_ && gui_);
  auto & g = *gui_;

  GraphVisualizationParams p;
  {
    auto lck = mrpt::lockHelper(g.params_mtx);
    p = g.params;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool styleChanged = g.params_changed.exchange(false);
  if (
    !styleChanged && g.last_update &&
    std::chrono::duration<double>(now - *g.last_update).count() < p.update_period) {
    return;
  }

  if (!g.window_created) {
    gui_build_window();
    g.window_created = true;
  }

  // Copy the graph only if it changed, then build the scene off the lock:
  GraphVisualizationInput in;
  std::vector<FrontEndKeyframePose> frontEnds;
  bool redraw = false;
  std::size_t numKeyframes = 0;
  std::size_t numLoops = 0;
  std::size_t rejectedByOdometry = 0;
  std::size_t rejectedByConsistency = 0;
  std::size_t gnssFactors = 0;
  std::size_t gravityFactors = 0;
  std::size_t gyroFactors = 0;
  std::size_t wheelFactors = 0;
  bool georeferenced = false;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    numKeyframes = keyframes_.size();
    numLoops = loops_.size();
    rejectedByOdometry = rejected_loop_pairs_.size();
    rejectedByConsistency = consistency_rejected_pairs_.size();
    gnssFactors = last_num_gnss_factors_;
    gravityFactors = last_num_gravity_factors_;
    gyroFactors = last_num_gyro_factors_;
    wheelFactors = last_num_wheel_odom_factors_;
    georeferenced = georef_.has_value();

    for (std::size_t i = g.kf_chain.size(); i < numKeyframes; i++) {
      const auto [it, isNew] =
        g.chain_ids.try_emplace(keyframes_[i].source_frame, g.chain_ids.size());
      if (isNew) {
        g.chain_newest_kf.push_back(i);
      }
      g.kf_chain.push_back(it->second);
      g.chain_newest_kf[it->second] = i;
    }

    const Gui::RedrawKey key = {
      numKeyframes, solves_.load(), loops_revision_, rejectedByOdometry, rejectedByConsistency};
    redraw = styleChanged || key != g.last_drawn;
    if (redraw) {
      g.last_drawn = key;
      in.keyframes.reserve(numKeyframes);
      for (const auto & kf : keyframes_) {
        in.keyframes.push_back(kf.pose_in_map.translation());
      }
      in.chain = g.kf_chain;
      in.loops.reserve(numLoops);
      for (const auto & l : loops_) {
        in.loops.emplace_back(l.from, l.to);
      }
      in.rejected_by_consistency = consistency_rejected_pairs_;
      in.rejected_by_odometry.assign(rejected_loop_pairs_.begin(), rejected_loop_pairs_.end());
      for (const auto i : g.chain_newest_kf) {
        const auto & kf = keyframes_[i];
        frontEnds.push_back({kf.source_frame, kf.pose_in_map, kf.pose_in_source.mean});
      }
    }
  }
  g.last_update = now;

  if (redraw) {
    visualizer_->update_3d_object("mapper/graph", build_graph_visualization(in, p));
    // Not aligned: back to their own frames.
    for (const auto & [name, pose] : front_end_viz_frame_poses(frontEnds)) {
      const auto framePose = p.align_front_end_frames ? pose : mrpt::poses::CPose3D();
      visualizer_->update_3d_object_frame(name, framePose.asTPose());
    }
  }

  g.lbKeyframes->set(
    mrpt::format("Keyframes: %zu (%zu front end chains)", numKeyframes, g.chain_ids.size()));
  g.lbLoops->set(
    mrpt::format("Loop closures: %zu accepted, %zu scans", numLoops, lc_scans_.load()));
  g.lbRejected->set(mrpt::format(
    "Rejected: %zu by odometry, %zu by consistency", rejectedByOdometry, rejectedByConsistency));
  g.lbFactors->set(mrpt::format(
    "Factors: GNSS %zu, gravity %zu, gyro %zu, wheels %zu", gnssFactors, gravityFactors,
    gyroFactors, wheelFactors));
  g.lbSolver->set(mrpt::format(
    "Solves: %zu, last %.0f ms; geo-referenced: %s", solves_.load(),
    1e3 * last_solve_seconds_.load(), georeferenced ? "yes" : "no"));
}

}  // namespace mola::mapper
