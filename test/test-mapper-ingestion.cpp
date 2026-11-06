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
 * @file   test-mapper-ingestion.cpp
 * @brief  Unit tests of the Mapper module itself: what it attaches to each
 *         keyframe (GNSS, gravity, gyroscope rotation), the feedback to the state
 *         estimator, and the end of run outputs.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace mola
{
/** A state estimator that only counts the poses fused into it. */
class MockNavState : public NavStateFilter
{
  DEFINE_MRPT_OBJECT(MockNavState, mola)
public:
  void reset() override {}
  void spinOnce() override {}
  void onNewObservation(const CObservation::ConstPtr & /*o*/) override {}
  void fuse_pose(
    const mrpt::Clock::time_point &, const mrpt::poses::CPose3DPDFGaussian &,
    const std::string &) override
  {
    fused_poses++;
  }
  void fuse_odometry(const mrpt::obs::CObservationOdometry &, const std::string &) override {}
  void fuse_imu(const mrpt::obs::CObservationIMU &) override {}
  void fuse_gnss(const mrpt::obs::CObservationGPS &) override {}
  void fuse_twist(
    const mrpt::Clock::time_point &, const mrpt::math::TTwist3D &,
    const mrpt::math::CMatrixDouble66 &) override
  {
  }
  std::optional<NavState> estimated_navstate(
    const mrpt::Clock::time_point &, const std::string &) override
  {
    return std::nullopt;
  }

  std::atomic<int> fused_poses{0};
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockNavState, mola::NavStateFilter, mola)

namespace
{
using mrpt::poses::CPose3D;

constexpr double T0 = 1.8e9;  // a realistic epoch [s]

mrpt::Clock::time_point at(double t) { return mrpt::Clock::fromDouble(T0 + t); }

std::shared_ptr<mola::mapper::Mapper> make_mapper(const std::string & extraParams)
{
  auto m = std::make_shared<mola::mapper::Mapper>();
  m->setMinLoggingLevel(mrpt::system::LVL_WARN);
  const std::string yaml = std::string(R"###(
params:
  reference_frame_name: "map"
  enable_optimizer_thread: false
  loop_closure_enabled: false
  gnss_sensor_labels_re: "gps"
  gnss_max_time_offset: 0.5
  imu_gravity_window: 1.0
  imu_gravity_min_samples: 10
)###") + extraParams;
  m->initialize(mrpt::containers::yaml::FromText(yaml));
  return m;
}

void insert_keyframe(mola::mapper::Mapper & m, double t, const CPose3D & poseInOdom)
{
  mola::SharedKeyframeMap::KeyframeInsertRequest req;
  req.timestamp = at(t);
  req.source_frame_id = "lidar_odom_kf";
  req.pose_in_source.mean = poseInOdom;
  req.pose_in_source.cov.setDiagonal(1e-6);
  ASSERT_(m.requestInsertKeyframe(req).has_value());
}

mrpt::obs::CObservationGPS::Ptr gnss(double t, bool gga)
{
  auto o = mrpt::obs::CObservationGPS::Create();
  o->sensorLabel = "gps";
  o->timestamp = at(t);
  if (gga) {
    mrpt::obs::gnss::Message_NMEA_GGA msg;
    msg.fields.latitude_degrees = 36.8287;
    msg.fields.longitude_degrees = -2.4073;
    msg.fields.altitude_meters = 50.0;
    msg.fields.fix_quality = 1;
    o->setMsg(msg);
  } else {
    mrpt::obs::gnss::Message_NMEA_RMC msg;
    msg.fields.latitude_degrees = 36.8287;
    msg.fields.longitude_degrees = -2.4073;
    msg.fields.validity_char = 'A';
    o->setMsg(msg);
  }
  return o;
}

/// GNSS readings attached to each keyframe of the central map, by their time [s].
std::vector<std::vector<double>> attached_gnss(const mola::mapper::Mapper & m)
{
  std::vector<std::vector<double>> out;
  const auto sm = m.simple_map();
  for (const auto & [pose, sf, twist] : sm) {
    out.emplace_back();
    for (const auto & o : *sf) {
      if (std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o)) {
        out.back().push_back(mrpt::Clock::toDouble(o->timestamp) - T0);
      }
    }
  }
  return out;
}

/** Each GNSS reading goes to the keyframe nearest in time (within the limit),
 *  only once a keyframe at or after it exists; a closer reading replaces it;
 *  readings without a position fix message (GGA) are ignored. */
void test_gnss_attachment()
{
  auto m = make_mapper("");
  insert_keyframe(*m, 0.0, CPose3D::Identity());
  m->onNewObservation(gnss(0.9, true));  // pending: no keyframe after it yet
  ASSERT_(attached_gnss(*m)[0].empty());

  insert_keyframe(*m, 1.0, CPose3D(1, 0, 0, 0, 0, 0));
  m->onNewObservation(gnss(1.05, true));   // closer to keyframe 1: replaces 0.9
  m->onNewObservation(gnss(1.01, false));  // even closer, but no GGA: ignored
  m->onNewObservation(gnss(1.30, true));   // farther than 1.05: dropped
  insert_keyframe(*m, 2.0, CPose3D(2, 0, 0, 0, 0, 0));
  m->onNewObservation(gnss(2.6, true));  // pending until keyframe 3
  m->onNewObservation(gnss(9.0, true));  // never near a keyframe
  insert_keyframe(*m, 3.0, CPose3D(3, 0, 0, 0, 0, 0));

  const auto g = attached_gnss(*m);
  ASSERT_EQUAL_(g.size(), 4U);
  ASSERT_(g[0].empty());
  ASSERT_EQUAL_(g[1].size(), 1U);
  ASSERT_NEAR_(g[1][0], 1.05, 1e-3);
  ASSERT_(g[2].empty());
  ASSERT_EQUAL_(g[3].size(), 1U);
  ASSERT_NEAR_(g[3][0], 2.6, 1e-3);
  std::cout << "GNSS attachment   OK\n";
}

mrpt::obs::CObservationImage::Ptr image(double t, const std::string & label = "cam")
{
  auto o = mrpt::obs::CObservationImage::Create();
  o->sensorLabel = label;
  o->timestamp = at(t);
  o->image = mrpt::img::CImage(16, 8, mrpt::img::CH_RGB);
  return o;
}

/// Images attached to each keyframe of the central map, by their time [s].
std::vector<std::vector<double>> attached_images(const mola::mapper::Mapper & m)
{
  std::vector<std::vector<double>> out;
  for (const auto & [pose, sf, twist] : m.simple_map()) {
    out.emplace_back();
    for (const auto & o : *sf) {
      if (std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o)) {
        out.back().push_back(mrpt::Clock::toDouble(o->timestamp) - T0);
      }
    }
  }
  return out;
}

/** Camera images follow the GNSS rule (nearest keyframe, a closer one replaces
 *  it), per camera. An image is written to a file, and replaced in the
 *  keyframe by a reference to it, once no closer one can come; the rest at
 *  the end. The input observations keep their pixels. */
void test_image_attachment()
{
  const std::string sm = mrpt::system::getTempFileName() + "_mapper.simplemap";
  const std::string dir = mrpt::system::fileNameChangeExtension(sm, "") + "_Images";
  auto m = make_mapper(
    "  image_sensor_labels_re: \"cam.*\"\n"
    "  image_max_time_offset: 0.5\n"
    "  image_format: png\n"
    "  output_simplemap: \"" +
    sm + "\"\n");
  std::vector<mrpt::obs::CObservationImage::Ptr> inputs;
  const auto feed = [&](double t, const std::string & label = "cam") {
    inputs.push_back(image(t, label));
    m->onNewObservation(inputs.back());
  };
  insert_keyframe(*m, 0.0, CPose3D::Identity());
  feed(0.9);  // pending: no keyframe after it yet
  insert_keyframe(*m, 1.0, CPose3D(1, 0, 0, 0, 0, 0));
  // All pending until keyframe 2:
  feed(1.02);           // closer to keyframe 1: replaces 0.9
  feed(1.03, "cam_r");  // another camera: its own slot
  feed(1.30);           // farther than 1.02: dropped
  insert_keyframe(*m, 2.0, CPose3D(2, 0, 0, 0, 0, 0));
  const auto file102 = mrpt::system::pathJoin({dir, mrpt::format("cam_%.09f.png", T0 + 1.02)});
  ASSERT_(!mrpt::system::fileExists(file102));
  feed(2.6);                                   // pending until keyframe 3
  ASSERT_(mrpt::system::fileExists(file102));  // a later image came: 1.02 is final
  insert_keyframe(*m, 3.0, CPose3D(3, 0, 0, 0, 0, 0));
  m->finalize();

  const auto g = attached_images(*m);
  ASSERT_EQUAL_(g.size(), 4U);
  ASSERT_(g[0].empty());
  ASSERT_EQUAL_(g[1].size(), 2U);
  ASSERT_(g[2].empty());
  ASSERT_EQUAL_(g[3].size(), 1U);
  ASSERT_NEAR_(g[3][0], 2.6, 1e-3);
  ASSERT_(mrpt::system::fileExists(
    mrpt::system::pathJoin({dir, mrpt::format("cam_r_%.09f.png", T0 + 1.03)})));
  ASSERT_(mrpt::system::fileExists(
    mrpt::system::pathJoin({dir, mrpt::format("cam_%.09f.png", T0 + 2.6)})));
  ASSERT_(!mrpt::system::fileExists(
    mrpt::system::pathJoin({dir, mrpt::format("cam_%.09f.png", T0 + 0.9)})));
  for (const auto & [pose, sf, twist] : m->simple_map()) {
    for (const auto & o : *sf) {
      if (const auto img = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o); img) {
        ASSERT_(img->image.isExternallyStored());
      }
    }
  }
  for (const auto & in : inputs) {
    ASSERT_(!in->image.isExternallyStored());
    ASSERT_EQUAL_(in->image.getWidth(), 16);
  }
  std::cout << "Image attachment  OK\n";
}

/** The gravity observation of a keyframe averages the accelerometer of one IMU
 *  only (the first one matching the regex), with its own sensor pose. */
void test_gravity_from_a_single_imu()
{
  auto m = make_mapper("  imu_sensor_labels_re: \"imu.*\"\n");
  const auto poseB = CPose3D(0, 0, 0, 0, mrpt::DEG2RAD(90.0), 0);
  for (int i = 0; i <= 200; i++) {
    const double t = i * 0.01;
    for (const char * label : {"imu_a", "imu_b"}) {
      auto o = mrpt::obs::CObservationIMU::Create();
      o->sensorLabel = label;
      o->timestamp = at(t);
      const bool isA = std::string(label) == "imu_a";
      o->sensorPose = isA ? CPose3D::Identity() : poseB;
      // The same gravity, seen from each mounting:
      o->set(mrpt::obs::IMU_X_ACC, isA ? 0.0 : -9.81);
      o->set(mrpt::obs::IMU_Y_ACC, 0.0);
      o->set(mrpt::obs::IMU_Z_ACC, isA ? 9.81 : 0.0);
      m->onNewObservation(o);
    }
  }
  insert_keyframe(*m, 1.5, CPose3D::Identity());

  const auto sm = m->simple_map();
  const auto & [pose, sf, twist] = *sm.begin();
  int n = 0;
  for (const auto & o : *sf) {
    if (auto imu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o); imu) {
      n++;
      ASSERT_EQUAL_(imu->sensorLabel, std::string("imu_a"));
      ASSERT_NEAR_(imu->get(mrpt::obs::IMU_X_ACC), 0.0, 1e-9);
      ASSERT_NEAR_(imu->get(mrpt::obs::IMU_Z_ACC), 9.81, 1e-9);
      ASSERT_NEAR_((imu->sensorPose - CPose3D::Identity()).translation().norm(), 0.0, 1e-9);
    }
  }
  ASSERT_EQUAL_(n, 1);
  std::cout << "Gravity from a single IMU   OK\n";
}

/** A keyframe inserted before the IMU readings of its window arrive (a lagging
 *  IMU stream) still gets its gravity, once, when they do. */
void test_gravity_when_imu_arrives_late()
{
  auto m = make_mapper("  imu_sensor_labels_re: \"imu.*\"\n");
  const auto feed = [&](double from, double to) {
    for (double t = from; t <= to + 1e-9; t += 0.01) {
      auto o = mrpt::obs::CObservationIMU::Create();
      o->sensorLabel = "imu_a";
      o->timestamp = at(t);
      o->set(mrpt::obs::IMU_X_ACC, 0.0);
      o->set(mrpt::obs::IMU_Y_ACC, 0.0);
      o->set(mrpt::obs::IMU_Z_ACC, 9.81);
      m->onNewObservation(o);
    }
  };
  feed(0.0, 0.2);  // known IMU, but not enough readings before the keyframe yet
  insert_keyframe(*m, 1.5, CPose3D::Identity());
  feed(0.21, 3.0);

  const auto count_gravity = [&]() {
    int n = 0;
    const auto sm = m->simple_map();
    for (const auto & o : *sm.begin()->sf) {
      if (std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o)) {
        n++;
      }
    }
    return n;
  };
  ASSERT_EQUAL_(count_gravity(), 1);
  feed(3.01, 4.0);
  ASSERT_EQUAL_(count_gravity(), 1);  // not attached twice
  std::cout << "Gravity with a lagging IMU   OK\n";
}

/** A 20 m square at 1 m/s, a keyframe per second, with LiDAR odometry losing
 *  2 deg per 90 deg turn; the gyroscope is mounted rolled 90 deg, so the vehicle
 *  yaw rate appears on another sensor axis. Returns the final heading error and
 *  the max roll/pitch error [deg]. */
std::pair<double, double> run_square_with_gyro(bool withGyro)
{
  auto m = make_mapper(
    std::string("  gnss_sensor_labels_re: \"\"\n  gyro_sensor_labels_re: \"") +
    (withGyro ? "imu" : "") + "\"\n  graph:\n    use_gnss: false\n    use_imu_gravity: false\n");
  const CPose3D sensorPose(0.1, 0, 0.2, 0, 0, mrpt::DEG2RAD(90.0));
  const auto Rvs = sensorPose.getRotationMatrix();

  CPose3D gt;
  CPose3D odo;
  std::vector<CPose3D> gtPoses;
  double t = 0;
  insert_keyframe(*m, t, odo);
  gtPoses.push_back(gt);
  for (int side = 0; side < 4; side++) {
    for (int i = 0; i < 20; i++) {
      const bool turn = (i == 19);
      // One second: 1 m forward, plus a 90 deg turn in place at the corners.
      const double wz = turn ? M_PI / 2 : 0.0;
      for (int k = 1; k <= 100; k++) {
        // Vehicle rate seen in the sensor frame: w_s = R_vs^T * w_v
        const Eigen::Vector3d ws = Rvs.asEigen().transpose() * Eigen::Vector3d(0.0, 0.0, wz);
        auto o = mrpt::obs::CObservationIMU::Create();
        o->sensorLabel = "imu";
        o->timestamp = at(t + 0.01 * k);
        o->sensorPose = sensorPose;
        o->set(mrpt::obs::IMU_WX, ws[0]);
        o->set(mrpt::obs::IMU_WY, ws[1]);
        o->set(mrpt::obs::IMU_WZ, ws[2]);
        m->onNewObservation(o);
      }
      t += 1.0;
      const CPose3D step(1.0, 0, 0, turn ? M_PI / 2 : 0.0, 0, 0);
      gt = gt + step;
      odo = odo + step + CPose3D(0, 0, 0, turn ? mrpt::DEG2RAD(2.0) : 0.0, 0, 0);
      insert_keyframe(*m, t, odo);
      gtPoses.push_back(gt);
    }
  }
  m->optimize_now();
  const auto est = m->keyframe_poses();
  ASSERT_EQUAL_(est.size(), gtPoses.size());
  double maxTilt = 0;
  for (std::size_t i = 0; i < est.size(); i++) {
    maxTilt = std::max(
      {maxTilt, std::abs(mrpt::math::wrapToPi(est[i].pitch() - gtPoses[i].pitch())),
       std::abs(mrpt::math::wrapToPi(est[i].roll() - gtPoses[i].roll()))});
  }
  const double yawErr = std::abs(mrpt::math::wrapToPi(est.back().yaw() - gtPoses.back().yaw()));
  return {mrpt::RAD2DEG(yawErr), mrpt::RAD2DEG(maxTilt)};
}

void test_gyro_rotation_in_vehicle_frame()
{
  const auto [yawNoGyro, tiltNoGyro] = run_square_with_gyro(false);
  const auto [yawGyro, tiltGyro] = run_square_with_gyro(true);
  std::cout << "Gyro (rolled mounting): final heading error " << yawNoGyro << " deg without, "
            << yawGyro << " deg with it; max roll/pitch error " << tiltGyro << " deg\n";
  ASSERT_GT_(yawNoGyro, 7.0);
  ASSERT_LT_(yawGyro, 2.0);
  ASSERT_LT_(tiltGyro, 0.5);
}

/** The newest keyframe pose is fed back to the state estimator once, however
 *  many solves happen before the next keyframe. */
void test_feedback_once_per_keyframe()
{
  auto m = std::make_shared<mola::mapper::Mapper>();
  auto se = std::make_shared<mola::MockNavState>();
  mola::MinimalModuleContainer container({m, se});
  m->setMinLoggingLevel(mrpt::system::LVL_WARN);
  m->initialize(mrpt::containers::yaml::FromText(R"###(
params:
  enable_optimizer_thread: false
  loop_closure_enabled: false
  gnss_sensor_labels_re: ""
  feedback_to_state_estimator: true
)###"));
  for (int i = 0; i < 3; i++) {
    insert_keyframe(*m, i, CPose3D(i, 0, 0, 0, 0, 0));
  }
  for (int k = 0; k < 3; k++) {
    m->optimize_now();
  }
  ASSERT_EQUAL_(se->fused_poses.load(), 1);
  insert_keyframe(*m, 3.0, CPose3D(3, 0, 0, 0, 0, 0));
  m->optimize_now();
  m->optimize_now();
  ASSERT_EQUAL_(se->fused_poses.load(), 2);
  std::cout << "Feedback once per keyframe   OK\n";
}

/** An output that cannot be written does not prevent the others. */
void test_outputs_survive_a_failing_one()
{
  const std::string tum = mrpt::system::getTempFileName() + "_mapper.tum";
  auto m = make_mapper(
    "  output_graph_input: \"/nonexistent_dir_mola_mapper/graph\"\n"
    "  output_loop_closures: \"/nonexistent_dir_mola_mapper/loops.txt\"\n"
    "  output_tum_trajectory: \"" +
    tum + "\"\n");
  for (int i = 0; i < 3; i++) {
    insert_keyframe(*m, i, CPose3D(i, 0, 0, 0, 0, 0));
  }
  m->finalize();
  ASSERT_(mrpt::system::fileExists(tum));
  mrpt::system::deleteFile(tum);
  std::cout << "Outputs survive a failing one   OK\n";
}

void test_loop_odometry_limits_loaded()
{
  auto m = make_mapper(
    "  loop_closure_odometry_check: false\n"
    "  loop_closure_odometry_limits:\n"
    "    max_xyz: 3.0\n"
    "    max_xyz_per_m: 0.2\n");
  const auto & p = m->parameters();
  ASSERT_(!p.loop_closure_odometry_check);
  ASSERT_NEAR_(p.loop_closure_odometry_limits.max_xyz, 3.0, 1e-12);
  ASSERT_NEAR_(p.loop_closure_odometry_limits.max_xyz_per_m, 0.2, 1e-12);
  ASSERT_NEAR_(p.loop_closure_odometry_limits.max_ang_deg, 5.0, 1e-12);  // default kept
  std::cout << "Loop odometry limits loaded   OK\n";
}

void test_unknown_robust_kernel_rejected_at_start()
{
  bool thrown = false;
  try {
    auto m = make_mapper("  graph:\n    gnss_robust_kernel: \"cauchyy\"\n");
  } catch (const std::exception &) {
    thrown = true;
  }
  ASSERT_(thrown);
  std::cout << "Unknown robust kernel rejected   OK\n";
}

}  // namespace

int main()
{
  try {
    test_gnss_attachment();
    test_image_attachment();
    test_gravity_from_a_single_imu();
    test_gravity_when_imu_arrives_late();
    test_gyro_rotation_in_vehicle_frame();
    test_feedback_once_per_keyframe();
    test_outputs_survive_a_failing_one();
    test_unknown_robust_kernel_rejected_at_start();
    test_loop_odometry_limits_loaded();
    std::cout << "All tests passed.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
