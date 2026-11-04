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
 * @file   test-mapper-replay.cpp
 * @brief  The mapper input recorded and replayed: the log reads back exactly,
 *         and two replays give the same map, on a simulated warehouse run with
 *         a drifting odometry and loop closures.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/Mapper.h>
#include <mola_mapper/MapperInputLog.h>
#include <mola_mapper/MapperReplay.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/system/filesystem.h>
#include <tbb/global_control.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace
{
using mrpt::poses::CPose3D;

std::string params_yaml(const std::string & recordFile, bool threads)
{
  const std::string t = threads ? "true" : "false";
  return std::string(R"###(
params:
  reference_frame_name: "map"
  enable_optimizer_thread: )###") +
         t + R"###(
  enable_loop_closure_thread: )###" +
         t + R"###(
  loop_closure_check_period_sec: 2.0
  gnss_sensor_labels_re: ""
  imu_sensor_labels_re: "imu"
  gyro_sensor_labels_re: "imu"
  loop_closure_enabled: true
  loop_closure_finalize_rounds: 1
  record_input: ")###" +
         recordFile + R"###("
  loop_closure_pipeline_file: ")###" +
         MOLA_MAPPER_LC_PIPELINE + "\"\n";
}

std::shared_ptr<mola::mapper::Mapper> make_mapper(const std::string & recordFile, bool threads)
{
  // The production detector pipeline, with candidates closer in time for this
  // short run, and no deskew (the simulated clouds have no point stamps):
  ::setenv("MIN_FRAMES_BETWEEN_LC", "20", 1);
  ::setenv("MOLA_DESKEW_METHOD", "MotionCompensationMethod::None", 1);
  auto m = std::make_shared<mola::mapper::Mapper>();
  m->setMinLoggingLevel(mrpt::system::LVL_WARN);
  m->initialize(mrpt::containers::yaml::FromText(params_yaml(recordFile, threads)));
  return m;
}

mrpt::obs::CObservationIMU::Ptr imu_reading(const mrpt::Clock::time_point & t, double wz)
{
  auto o = mrpt::obs::CObservationIMU::Create();
  o->sensorLabel = "imu";
  o->timestamp = t;
  o->set(mrpt::obs::IMU_X_ACC, 0.0);
  o->set(mrpt::obs::IMU_Y_ACC, 0.0);
  o->set(mrpt::obs::IMU_Z_ACC, 9.81);
  o->set(mrpt::obs::IMU_WX, 0.0);
  o->set(mrpt::obs::IMU_WY, 0.0);
  o->set(mrpt::obs::IMU_WZ, wz);
  return o;
}

/** A live-like run (with the mapper threads), recorded. Returns the requests sent. */
std::vector<mola::SharedKeyframeMap::KeyframeInsertRequest> record_run(const std::string & file)
{
  mrpt::maps::CSimpleMap sm;
  ASSERT_(sm.loadFromFile(MOLA_TEST_WAREHOUSE_SIMPLEMAP));
  std::vector<mola::SharedKeyframeMap::KeyframeInsertRequest> sent;

  auto m = make_mapper(file, true);
  CPose3D prevGt = sm.get(0).pose->getMeanVal();
  CPose3D odo = prevGt;
  mrpt::Clock::time_point prevStamp;
  for (std::size_t i = 0; i < sm.size(); i++) {
    const auto & sf = sm.get(i).sf;
    const CPose3D gt = sm.get(i).pose->getMeanVal();
    const auto stamp = sf->getObservationByIndex(0)->timestamp;
    if (i > 0) {
      odo = odo + (gt - prevGt) + CPose3D(0, 0, 0, mrpt::DEG2RAD(0.2), 0, 0);
      // IMU readings between keyframes, with the true yaw rate:
      const double dt = mrpt::system::timeDifference(prevStamp, stamp);
      const double wz = (gt - prevGt).yaw() / dt;
      for (int k = 1; k <= 20; k++) {
        m->onNewObservation(imu_reading(
          mrpt::Clock::fromDouble(mrpt::Clock::toDouble(prevStamp) + dt * k / 20.0), wz));
      }
    }
    mola::SharedKeyframeMap::KeyframeInsertRequest req;
    req.timestamp = stamp;
    req.source_frame_id = "lidar_odom_kf";
    req.pose_in_source.mean = odo;
    req.pose_in_source.cov.setDiagonal(1e-6 * static_cast<double>(i + 1));
    req.quality = 0.9;
    req.observations = *sf;
    ASSERT_(m->requestInsertKeyframe(req).has_value());
    sent.push_back(req);
    prevGt = gt;
    prevStamp = stamp;
  }
  m->finalize();  // closes the recording
  return sent;
}

struct ReplayResult
{
  std::vector<CPose3D> poses;
  std::size_t loops = 0;
  mola::mapper::ReplayStats stats;
};

ReplayResult replay(const std::string & file)
{
  auto m = make_mapper("", false);
  ReplayResult r;
  r.stats = mola::mapper::replay_mapper_input(*m, file);
  m->finalize();
  r.poses = m->keyframe_poses();
  r.loops = m->loop_closure_count();
  return r;
}

void test_record_and_replay()
{
  const std::string file = mrpt::system::getTempFileName() + "_mapper_input.gz";
  const auto sent = record_run(file);

  // The log reads back in order, with the exact requests:
  std::size_t nKf = 0;
  std::size_t nObs = 0;
  mola::mapper::MapperInputReader reader(file);
  while (auto e = reader.next()) {
    if (const auto * kf = std::get_if<mola::mapper::RecordedKeyframe>(&*e); kf) {
      ASSERT_LT_(nKf, sent.size());
      const auto & a = sent[nKf];
      const auto & b = kf->request;
      ASSERT_(a.timestamp == b.timestamp);
      ASSERT_EQUAL_(a.source_frame_id, b.source_frame_id);
      ASSERT_EQUAL_(a.quality, b.quality);
      ASSERT_(
        a.pose_in_source.mean.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>() ==
        b.pose_in_source.mean.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>());
      ASSERT_(a.pose_in_source.cov == b.pose_in_source.cov);
      ASSERT_EQUAL_(a.observations.size(), b.observations.size());
      nKf++;
    } else {
      nObs++;
    }
  }
  ASSERT_EQUAL_(nKf, sent.size());
  ASSERT_EQUAL_(nObs, (sent.size() - 1) * 20);

  // Two replays, the same result to the last bit:
  const auto r1 = replay(file);
  const auto r2 = replay(file);
  std::cout << "Replay: " << r1.stats.keyframes << " keyframes, " << r1.stats.observations
            << " observations, " << r1.stats.loop_closure_scans << " loop closure scans, "
            << r1.stats.solves << " solves, " << r1.loops << " loops\n";
  ASSERT_EQUAL_(r1.stats.keyframes, sent.size());
  ASSERT_GT_(r1.loops, 0U);
  ASSERT_GT_(r1.stats.loop_closure_scans, 1U);
  ASSERT_EQUAL_(r1.loops, r2.loops);
  ASSERT_EQUAL_(r1.poses.size(), r2.poses.size());
  double maxDiff = 0;
  for (std::size_t i = 0; i < r1.poses.size(); i++) {
    const auto d = (r1.poses[i].asVectorVal() - r2.poses[i].asVectorVal()).norm();
    maxDiff = std::max(maxDiff, d);
  }
  std::cout << "Max pose difference between the two replays: " << maxDiff << "\n";
  ASSERT_EQUAL_(maxDiff, 0.0);

  // A mapper with its threads is rejected:
  bool threw = false;
  try {
    auto m = make_mapper("", true);
    mola::mapper::replay_mapper_input(*m, file);
  } catch (const std::exception &) {
    threw = true;
  }
  ASSERT_(threw);

  mrpt::system::deleteFile(file);
}

}  // namespace

int main()
{
  // As mola-mapper-cli: parallel sums in ICP and the solver change the last
  // bits between runs with several threads.
  const tbb::global_control oneThread(tbb::global_control::max_allowed_parallelism, 1);
  try {
    test_record_and_replay();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
