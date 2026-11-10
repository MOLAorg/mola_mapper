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

#include <atomic>
#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <tuple>

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
  /// Keyframes, solves and loop revision of the last drawn graph.
  std::tuple<std::size_t, std::size_t, std::size_t> last_drawn{0, 0, 0};
};

namespace
{
constexpr const char * KF_SUFFIX = "_kf";

/// The frame a front end draws in, from the source frame of its keyframes.
std::optional<std::string> front_end_viz_frame(const std::string & sourceFrame)
{
  const std::string suffix = KF_SUFFIX;
  if (
    sourceFrame.size() <= suffix.size() ||
    sourceFrame.compare(sourceFrame.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return std::nullopt;
  }
  return sourceFrame.substr(0, sourceFrame.size() - suffix.size());
}
}  // namespace

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
    if (params_.loop_closure_enabled) {
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

  visualizer_->create_subwindow_from_description(desc).get();
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

  // Copy what is shown (cheap), then build the scene off the lock:
  GraphVisualizationInput in;
  std::map<std::string, mrpt::poses::CPose3D> frontEndFrames;
  std::size_t rejectedByOdometry = 0;
  std::size_t gnssFactors = 0;
  std::size_t gravityFactors = 0;
  std::size_t gyroFactors = 0;
  std::size_t wheelFactors = 0;
  bool georeferenced = false;
  std::size_t numChains = 0;
  std::tuple<std::size_t, std::size_t, std::size_t> drawn;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    drawn = {keyframes_.size(), solves_.load(), loops_revision_};
    std::map<std::string, std::size_t> chainIds;
    in.keyframes.reserve(keyframes_.size());
    in.chain.reserve(keyframes_.size());
    for (const auto & kf : keyframes_) {
      in.keyframes.push_back(kf.pose_in_map.translation());
      in.chain.push_back(chainIds.emplace(kf.source_frame, chainIds.size()).first->second);
      if (const auto f = front_end_viz_frame(kf.source_frame); f) {
        // The newest keyframe of each chain sets its frame: T_map_source.
        frontEndFrames[*f] = kf.pose_in_map + (mrpt::poses::CPose3D() - kf.pose_in_source.mean);
      }
    }
    numChains = chainIds.size();
    for (const auto & l : loops_) {
      in.loops.emplace_back(l.from, l.to);
    }
    in.rejected_by_consistency = consistency_rejected_pairs_;
    in.rejected_by_odometry.assign(rejected_loop_pairs_.begin(), rejected_loop_pairs_.end());
    rejectedByOdometry = rejected_loop_pairs_.size();
    gnssFactors = last_num_gnss_factors_;
    gravityFactors = last_num_gravity_factors_;
    gyroFactors = last_num_gyro_factors_;
    wheelFactors = last_num_wheel_odom_factors_;
    georeferenced = georef_.has_value();
  }
  g.last_update = now;

  // The scene only if the graph or the style changed:
  if (styleChanged || drawn != g.last_drawn) {
    g.last_drawn = drawn;
    visualizer_->update_3d_object("mapper/graph", build_graph_visualization(in, p));
    // Not aligned: back to their own frames.
    for (const auto & [name, pose] : frontEndFrames) {
      const auto framePose = p.align_front_end_frames ? pose : mrpt::poses::CPose3D();
      visualizer_->update_3d_object_frame(name, framePose.asTPose());
    }
  }

  g.lbKeyframes->set(
    mrpt::format("Keyframes: %zu (%zu front end chains)", in.keyframes.size(), numChains));
  g.lbLoops->set(
    mrpt::format("Loop closures: %zu accepted, %zu scans", in.loops.size(), lc_scans_.load()));
  g.lbRejected->set(mrpt::format(
    "Rejected: %zu by odometry, %zu by consistency", rejectedByOdometry,
    in.rejected_by_consistency.size()));
  g.lbFactors->set(mrpt::format(
    "Factors: GNSS %zu, gravity %zu, gyro %zu, wheels %zu", gnssFactors, gravityFactors,
    gyroFactors, wheelFactors));
  g.lbSolver->set(mrpt::format(
    "Solves: %zu, last %.0f ms; geo-referenced: %s", solves_.load(),
    1e3 * last_solve_seconds_.load(), georeferenced ? "yes" : "no"));
}

}  // namespace mola::mapper
