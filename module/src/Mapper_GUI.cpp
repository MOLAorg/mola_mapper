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
#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_mapper/CameraDirector.h>
#include <mola_mapper/KeyframeMapVisualization.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/viz/Scene.h>

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace mola::mapper
{
namespace
{
// Time spent building keyframe clouds per GUI update [s]: a long backlog (e.g.
// the map enabled late in a run) is built over several updates.
constexpr double KEYFRAME_MAP_BUILD_BUDGET = 0.3;
}  // namespace

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
  mola::gui::LiveString::Ptr lbKeyframeMap = std::make_shared<mola::gui::LiveString>(" ");

  // The global map of keyframe clouds, created when first shown:
  std::unique_ptr<KeyframeMapVisualization> keyframe_map;
  bool keyframe_map_drawn = false;

  // Scripted camera: its own thread (smooth motion at its own rate), started
  // when first enabled. Its inputs: the map extent, newest keyframe and the
  // end of the data (gui_update()), the scene frame of the front end and its
  // pose in the map (T_map_f), and the latest data stamp, to ask the state
  // estimator for the current vehicle pose in that frame.
  std::thread camera_thread;
  std::atomic_bool camera_stop{false};
  std::mutex camera_mtx;
  CameraDirector::Inputs camera_inputs;
  std::optional<std::pair<std::string, mrpt::poses::CPose3D>> front_end_frame;
  std::atomic<double> last_data_stamp{0};
  std::shared_ptr<mola::NavStateFilter> state_estimator;
  std::shared_ptr<mola::Dataset_UI> dataset;

  Gui() = default;
  Gui(const Gui &) = delete;
  Gui & operator=(const Gui &) = delete;
  Gui(Gui &&) = delete;
  Gui & operator=(Gui &&) = delete;
  ~Gui()
  {
    camera_stop = true;
    if (camera_thread.joinable()) {
      camera_thread.join();
    }
  }

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
  // For the scripted camera: the current vehicle pose and the data progress.
  if (const auto se = findService<mola::NavStateFilter>(); !se.empty()) {
    gui_->state_estimator = std::dynamic_pointer_cast<mola::NavStateFilter>(se.front());
  }
  if (const auto ds = findService<mola::Dataset_UI>(); !ds.empty()) {
    gui_->dataset = std::dynamic_pointer_cast<mola::Dataset_UI>(ds.front());
  }
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
    tab.widgets.emplace_back(Label{g->lbKeyframeMap});
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

    SliderFloat odoMaxLength = radius;
    odoMaxLength.label = "Odometry edges, max length [m]";
    odoMaxLength.initial_value = p.odometry_edge_max_length;
    odoMaxLength.max_value = 100.0f;
    odoMaxLength.format_string = "%.0f";
    odoMaxLength.on_change = setter(&GraphVisualizationParams::odometry_edge_max_length);
    tab.widgets.emplace_back(odoMaxLength);

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

    tab.widgets.emplace_back(Separator{});
    tab.widgets.emplace_back(CheckBox{
      "Show the map (keyframe clouds)", p.show_keyframe_map,
      setter(&GraphVisualizationParams::show_keyframe_map)});
    SliderFloat mapPointSize = odoWidth;
    mapPointSize.label = "Map point size [px]";
    mapPointSize.initial_value = p.keyframe_map.point_size;
    mapPointSize.min_value = 1.0f;
    mapPointSize.max_value = 8.0f;
    mapPointSize.on_change = [g](float v) {
      {
        auto lck = mrpt::lockHelper(g->params_mtx);
        g->params.keyframe_map.point_size = v;
      }
      g->params_changed = true;
    };
    tab.widgets.emplace_back(mapPointSize);

    tab.widgets.emplace_back(Separator{});
    tab.widgets.emplace_back(
      CheckBox{"Scripted camera (map overview, close-ups)", p.camera.enabled, [g](bool v) {
                 {
                   auto lck = mrpt::lockHelper(g->params_mtx);
                   g->params.camera.enabled = v;
                 }
                 g->params_changed = true;
               }});
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
  if (p.camera.enabled) {
    gui_feed_camera(p);
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

  const bool showMap = p.show_keyframe_map;
  if (showMap && !g.keyframe_map) {
    g.keyframe_map = std::make_unique<KeyframeMapVisualization>(p.keyframe_map);
  }

  // Copy the graph only if it changed, then build the scene off the lock:
  GraphVisualizationInput in;
  std::vector<mrpt::poses::CPose3D> poses;
  std::vector<KeyframeMapVisualization::KeyframeData> newKeyframes;
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
    // Keyframes not yet considered for the map (shallow copies):
    const bool mapPending = showMap && g.keyframe_map->num_considered() < numKeyframes;
    if (mapPending) {
      for (std::size_t i = g.keyframe_map->num_considered(); i < numKeyframes; i++) {
        newKeyframes.push_back({i, keyframes_[i].observations});
      }
    }
    if (redraw || mapPending) {
      poses.reserve(numKeyframes);
      for (const auto & kf : keyframes_) {
        poses.push_back(kf.pose_in_map);
      }
    }
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
      auto lck = mrpt::lockHelper(g.camera_mtx);
      g.front_end_frame = {name, pose};
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
  if (g.keyframe_map) {
    g.lbKeyframeMap->set(mrpt::format(
      "Map: %zu keyframe clouds, %.02f M points%s", g.keyframe_map->num_clouds(),
      1e-6 * static_cast<double>(g.keyframe_map->num_points()),
      g.keyframe_map->num_considered() < numKeyframes ? " (building)" : ""));
  } else {
    g.lbKeyframeMap->set("Map: not shown");
  }
}

void Mapper::gui_feed_camera(const GraphVisualizationParams & p)
{
  // The scripted camera runs at its own rate: feed it the map extent, the
  // newest keyframe (vehicle pose if the state estimator has none) and
  // whether the data is about to end.
  auto & g = *gui_;
  CameraDirector::Inputs in;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (!keyframes_.empty()) {
      auto box = mrpt::math::TBoundingBox::PlusMinusInfinity();
      for (const auto & kf : keyframes_) {
        box.updateWithPoint(kf.pose_in_map.translation());
      }
      in.map = box;
      in.vehicle = keyframes_.back().pose_in_map.translation();
      in.vehicle_yaw = keyframes_.back().pose_in_map.yaw();
    }
  }
#if defined(MOLA_KERNEL_DATASET_UI_HAS_TIME)
  if (g.dataset) {
    const auto tNow = g.dataset->datasetUI_time();
    const auto tEnd = g.dataset->datasetUI_total_time();
    in.final = tNow && tEnd && *tEnd - *tNow < p.camera.final_overview;
  }
#endif
  {
    auto lck = mrpt::lockHelper(g.camera_mtx);
    g.camera_inputs = in;
  }
  if (!g.camera_thread.joinable()) {
    gui_start_camera(p.camera);
  }
}

void Mapper::gui_draw_keyframe_map(
  const GraphVisualizationParams & p, bool redraw,
  const std::vector<KeyframeMapVisualization::KeyframeData> & newKeyframes,
  const std::vector<mrpt::poses::CPose3D> & poses)
{
  auto & g = *gui_;
  const bool showMap = p.show_keyframe_map;
  // The map, off the lock: new clouds, then the scene if anything changed.
  bool mapChanged = false;
  if (showMap && !newKeyframes.empty()) {
    mapChanged = g.keyframe_map->add_keyframes(newKeyframes, poses, KEYFRAME_MAP_BUILD_BUDGET) > 0;
  }
  if (g.keyframe_map) {
    const float pointSize = p.keyframe_map.point_size;
    if (pointSize != g.keyframe_map->point_size()) {
      g.keyframe_map->set_point_size(pointSize);
      // Clouds already in the scene: changed from the render thread.
      visualizer_->execute_custom_code_on_background_scene(
        [clouds = g.keyframe_map->clouds(), pointSize](mrpt::viz::Scene &) {
          for (const auto & c : clouds) {
            c->setPointSize(pointSize);
          }
        });
    }
  }
  if (showMap && (redraw || mapChanged)) {
    visualizer_->update_3d_object("mapper/keyframe_map", g.keyframe_map->build_scene(poses));
    g.keyframe_map_drawn = true;
  } else if (!showMap && g.keyframe_map_drawn) {
    visualizer_->update_3d_object("mapper/keyframe_map", mrpt::viz::CSetOfObjects::Create());
    g.keyframe_map_drawn = false;
  }
}

void Mapper::gui_start_camera(const CameraDirectorParams & cp)
{
  auto & g = *gui_;
  MRPT_LOG_INFO_STREAM(
    "Scripted camera at " << cp.rate << " Hz, vehicle pose from "
                          << (g.state_estimator ? "the state estimator" : "the keyframes"));
  g.camera_stop = false;
  // Only the Gui and the modules it holds are used from the thread: it is
  // joined before they are destroyed (gui_stop_camera(), ~Gui()).
  g.camera_thread = std::thread([gp = gui_.get(), viz = visualizer_, cp]() {
    using clock = std::chrono::steady_clock;
    CameraDirector director(cp);
    const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / std::max(cp.rate, 1.0)));
    const auto t0 = clock::now();
    auto next = t0;
    while (!gp->camera_stop) {
      next += period;
      std::this_thread::sleep_until(next);
      if (clock::now() > next + period) {
        next = clock::now();  // fell behind: no burst of updates
      }
      bool enabled = false;
      {
        auto lck = mrpt::lockHelper(gp->params_mtx);
        enabled = gp->params.camera.enabled;
      }
      if (!enabled) {
        continue;
      }
      CameraDirector::Inputs in;
      std::optional<std::pair<std::string, mrpt::poses::CPose3D>> frontEnd;
      {
        auto lck = mrpt::lockHelper(gp->camera_mtx);
        in = gp->camera_inputs;
        frontEnd = gp->front_end_frame;
      }
      // The current vehicle pose, smoother than the keyframes (none of them
      // while revisiting mapped areas):
      const double stamp = gp->last_data_stamp;
      if (gp->state_estimator && frontEnd && stamp > 0) {
        if (const auto ns = gp->state_estimator->estimated_navstate(
              mrpt::Clock::fromDouble(stamp), frontEnd->first);
            ns) {
          const auto pose = frontEnd->second + ns->pose.mean;
          in.vehicle = pose.translation();
          in.vehicle_yaw = pose.yaw();
        }
      }
      const auto c = director.step(std::chrono::duration<double>(clock::now() - t0).count(), in);
      if (!c) {
        continue;
      }
      const mrpt::math::TPoint3Df lookAt(c->look_at);
#if defined(MOLA_KERNEL_VIZ_HAS_CAMERA_ORIENTATION)
      viz->update_viewport_camera(lookAt, c->azimuth, c->elevation, c->distance);
#else
      // Older visualizers: look-at and azimuth only.
      viz->update_viewport_look_at(lookAt);
      viz->update_viewport_camera_azimuth(c->azimuth);
#endif
    }
  });
}

void Mapper::gui_on_new_data(const mrpt::Clock::time_point & stamp)
{
  if (!gui_) {
    return;
  }
  const double t = mrpt::Clock::toDouble(stamp);
  // Newest only: sources may interleave slightly out of order.
  double prev = gui_->last_data_stamp.load();
  while (t > prev && !gui_->last_data_stamp.compare_exchange_weak(prev, t)) {
  }
}

void Mapper::gui_stop_camera()
{
  if (!gui_ || !gui_->camera_thread.joinable()) {
    return;
  }
  gui_->camera_stop = true;
  gui_->camera_thread.join();
}

}  // namespace mola::mapper
