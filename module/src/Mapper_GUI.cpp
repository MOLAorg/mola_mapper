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
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_mapper/CameraDirector.h>
#include <mola_mapper/KeyframeMapVisualization.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
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
// Keyframes copied per GUI update for the map, at most:
constexpr std::size_t KEYFRAME_MAP_MAX_BATCH = 100;
// A correction shot shows the region within this distance [m] of the node
// that moved most:
constexpr double CORRECTION_SHOT_RADIUS = 80.0;
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

  // GUI animation thread (scripted camera, map glides; smooth motion at its
  // own rate), started when either is first enabled. Its inputs (camera_mtx): the map extent, newest
  // keyframe and the end of the data (gui_update()), the scene frame of the
  // front end and its pose in the map as drawn (T_map_f), and the latest
  // vehicle pose advertised in each frame by the localization sources.
  std::thread animation_thread;
  std::atomic_bool animation_stop{false};
  std::mutex camera_mtx;
  CameraDirector::Inputs camera_inputs;
  std::optional<std::pair<std::string, mrpt::poses::CPose3D>> front_end_frame;
  std::map<std::string, mrpt::poses::CPose3D> vehicle_poses;
  std::weak_ptr<mola::Dataset_UI> dataset;

  // Map corrections to glide into place (and maybe to show in a camera
  // shot), from gui_draw_keyframe_map() to the animation thread (camera_mtx):
  struct Correction
  {
    std::vector<KeyframeMapVisualization::NodePose> changes;
    std::optional<mrpt::math::TBoundingBox> shot_region;
    double max_shift = 0;
    double glide = 0;  //!< [s]
  };
  std::vector<Correction> pending_corrections;

  Gui() = default;
  Gui(const Gui &) = delete;
  Gui & operator=(const Gui &) = delete;
  Gui(Gui &&) = delete;
  Gui & operator=(Gui &&) = delete;
  ~Gui()
  {
    animation_stop = true;
    if (animation_thread.joinable()) {
      animation_thread.join();
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
  gui_stop_animation();
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
  // For the scripted camera: the current vehicle pose (passively, from the
  // localization updates of the front ends) and the data progress.
  for (const auto & m : findService<mola::LocalizationSourceBase>()) {
    auto src = std::dynamic_pointer_cast<mola::LocalizationSourceBase>(m);
    if (!src) {
      continue;
    }
    src->subscribeToLocalizationUpdates(
      [weakGui =
         std::weak_ptr<Gui>(gui_)](const mola::LocalizationSourceBase::LocalizationUpdate & u) {
        if (auto gp = weakGui.lock(); gp) {
          auto lck = mrpt::lockHelper(gp->camera_mtx);
          gp->vehicle_poses[u.reference_frame] = mrpt::poses::CPose3D(u.pose);
        }
      });
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
  // The animation thread runs the scripted camera and the map glides:
  if (
    (p.camera.enabled || (p.show_keyframe_map && p.keyframe_map.glide > 0)) &&
    !g.animation_thread.joinable()) {
    gui_start_animation(p);
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
      // A long backlog (e.g. the map enabled late) is built over several updates:
      const std::size_t last =
        std::min(numKeyframes, g.keyframe_map->num_considered() + KEYFRAME_MAP_MAX_BATCH);
      for (std::size_t i = g.keyframe_map->num_considered(); i < last; i++) {
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

  gui_draw_keyframe_map(p, redraw, newKeyframes, poses);

  if (redraw) {
    visualizer_->update_3d_object("mapper/graph", build_graph_visualization(in, p));
    // Not aligned: back to their own frames.
    for (const auto & [name, pose] : front_end_viz_frame_poses(frontEnds)) {
      const auto framePose = p.align_front_end_frames ? pose : mrpt::poses::CPose3D();
      visualizer_->update_3d_object_frame(name, framePose.asTPose());
      // The camera follows the vehicle as drawn:
      auto lck = mrpt::lockHelper(g.camera_mtx);
      g.front_end_frame = {name, framePose};
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
  if (const auto ds = g.dataset.lock(); ds) {
    const auto tNow = ds->datasetUI_time();
    const auto tEnd = ds->datasetUI_total_time();
    in.final = tNow && tEnd && *tEnd - *tNow < p.camera.final_overview;
  }
#endif
  {
    auto lck = mrpt::lockHelper(g.camera_mtx);
    g.camera_inputs = in;
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
  if (showMap && (mapChanged || !g.keyframe_map_drawn)) {
    // The same nodes as before plus the new ones: the visualizer keeps the
    // GPU buffers of those it already had.
    visualizer_->update_3d_object("mapper/keyframe_map", g.keyframe_map->scene());
    g.keyframe_map_drawn = true;
  }
  if (showMap && redraw) {
    // Keyframes moved (e.g. a loop closure): move their nodes, in the render
    // thread, at once or gliding (animation thread).
    auto moved = g.keyframe_map->pose_changes(poses);
    if (!moved.empty() && p.keyframe_map.glide > 0) {
      gui_queue_correction(p, std::move(moved));
    } else if (!moved.empty()) {
      visualizer_->execute_custom_code_on_background_scene(
        [moved = std::move(moved)](mrpt::viz::Scene &) {
          for (const auto & m : moved) {
            m.node->setPose(m.pose);
          }
        });
    }
  } else if (!showMap && g.keyframe_map_drawn) {
    visualizer_->update_3d_object("mapper/keyframe_map", mrpt::viz::CSetOfObjects::Create());
    g.keyframe_map_drawn = false;
  }
}

void Mapper::gui_queue_correction(
  const GraphVisualizationParams & p, std::vector<KeyframeMapVisualization::NodePose> && moved)
{
  // Where the map moved most: a camera shot of large corrections.
  Gui::Correction c;
  c.glide = p.keyframe_map.glide;
  if (p.camera.enabled && p.camera.shot_min_correction > 0) {
    c.shot_region =
      correction_region(moved, p.camera.shot_min_correction, CORRECTION_SHOT_RADIUS, &c.max_shift);
    if (c.shot_region) {
      MRPT_LOG_DEBUG_STREAM("Map correction of up to " << c.max_shift << " m: shot candidate");
    }
  }
  c.changes = std::move(moved);
  auto lck = mrpt::lockHelper(gui_->camera_mtx);
  gui_->pending_corrections.push_back(std::move(c));
}

void Mapper::gui_start_animation(const GraphVisualizationParams & p)
{
  auto & g = *gui_;
  const auto & cp = p.camera;
  MRPT_LOG_INFO_STREAM(
    "GUI animation thread at " << cp.rate << " Hz (scripted camera: " << (cp.enabled ? "on" : "off")
                               << ", map glides: " << p.keyframe_map.glide << " s)");
  g.animation_stop = false;
  // Only the Gui and the visualizer are used from the thread: it is joined
  // before they are destroyed (gui_stop_animation(), ~Gui()).
  g.animation_thread = std::thread([gp = gui_.get(), viz = visualizer_, cp]() {
    using clock = std::chrono::steady_clock;
    CameraDirector director(cp);
    NodeAnimator animator;
    const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / std::max(cp.rate, 1.0)));
    const auto t0 = clock::now();
    auto next = t0;
    double farClip = 1000.0;  // [m] MRPT default
    std::ofstream timeline;
    if (!cp.timeline_file.empty()) {
      timeline.open(cp.timeline_file);
      if (!timeline.is_open()) {
        std::cerr << "[mola_mapper] Cannot write the camera timeline: " << cp.timeline_file << "\n";
      }
      timeline << "# unix_time[s] closeup_weight shot_weight (space separated)\n"
               << "# events as comments: '# shot <unix_time> <max_correction[m]>'\n";
    }
    // Pose updates for the GUI thread: one at a time (they are a function of
    // time, so skipping a tick while the GUI is busy loses nothing).
    std::future<bool> posesTask;
    double lastTimelineRow = -1;
    const auto unixNow = []() {
      return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
    };

    while (!gp->animation_stop) {
      try {
        next += period;
        std::this_thread::sleep_until(next);
        if (clock::now() > next + period) {
          next = clock::now();  // fell behind: no burst of updates
        }
        const double t = std::chrono::duration<double>(clock::now() - t0).count();
        bool cameraOn = false;
        {
          auto lck = mrpt::lockHelper(gp->params_mtx);
          cameraOn = gp->params.camera.enabled;
        }

        // Map corrections: glide now, or once a camera shot is there.
        std::vector<Gui::Correction> corrections;
        CameraDirector::Inputs in;
        {
          auto lck = mrpt::lockHelper(gp->camera_mtx);
          corrections = std::move(gp->pending_corrections);
          gp->pending_corrections.clear();
          in = gp->camera_inputs;
          // The current vehicle pose, more frequent than the keyframes (none
          // while revisiting mapped areas), mapped through T_map_f:
          if (const auto & fe = gp->front_end_frame; fe) {
            if (const auto it = gp->vehicle_poses.find(fe->first); it != gp->vehicle_poses.end()) {
              const auto pose = fe->second + it->second;
              in.vehicle = pose.translation();
              in.vehicle_yaw = pose.yaw();
            }
          }
        }
        for (const auto & c : corrections) {
          double start = t;
          if (cameraOn && c.shot_region) {
            if (const auto arrival = director.start_shot(t, *c.shot_region); arrival) {
              start = *arrival;
              if (timeline.is_open()) {
                timeline << mrpt::format("# shot %.3f %.3f\n", unixNow(), c.max_shift);
              }
            }
          }
          animator.add(c.changes, start, c.glide, t);
        }
        const bool guiReady = !posesTask.valid() || posesTask.wait_for(std::chrono::seconds(0)) ==
                                                      std::future_status::ready;
        if (guiReady && !animator.empty()) {
          if (auto poses = animator.step(t); !poses.empty()) {
            posesTask = viz->execute_custom_code_on_background_scene(
              [poses = std::move(poses)](mrpt::viz::Scene &) {
                for (const auto & m : poses) {
                  m.node->setPose(m.pose);
                }
              });
          }
        }

        if (!cameraOn) {
          continue;
        }
        const auto c = director.step(t, in);
        if (!c) {
          continue;
        }
        if (timeline.is_open() && t - lastTimelineRow >= 0.5) {
          lastTimelineRow = t;
          timeline << mrpt::format("%.3f %.3f %.3f\n", unixNow(), c->closeup, c->shot);
          timeline.flush();
        }
        // Large maps: push the far clip plane beyond the overview distance.
        if (2 * c->distance > farClip) {
          farClip = 4 * c->distance;
          viz->execute_custom_code_on_background_scene([farClip](mrpt::viz::Scene & scene) {
            if (auto vp = scene.getViewport("main"); vp) {
              float clipMin = 0;
              float clipMax = 0;
              vp->getViewportClipDistances(clipMin, clipMax);
              vp->setViewportClipDistances(clipMin, std::max(clipMax, static_cast<float>(farClip)));
            }
          });
        }
        const mrpt::math::TPoint3Df lookAt(c->look_at);
#if defined(MOLA_KERNEL_VIZ_HAS_CAMERA_ORIENTATION)
        viz->update_viewport_camera(lookAt, c->azimuth, c->elevation, c->distance);
#else
        // Older visualizers: look-at and azimuth only.
        viz->update_viewport_look_at(lookAt);
        viz->update_viewport_camera_azimuth(c->azimuth);
#endif
      } catch (const std::exception & e) {
        // A GUI failure must never stop the mapping:
        std::cerr << "[mola_mapper] GUI animation error: " << e.what() << "\n";
      }
    }
  });
}

void Mapper::gui_stop_animation()
{
  if (!gui_ || !gui_->animation_thread.joinable()) {
    return;
  }
  gui_->animation_stop = true;
  gui_->animation_thread.join();
}

}  // namespace mola::mapper
