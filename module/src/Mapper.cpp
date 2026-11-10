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
 * @file   Mapper.cpp
 * @brief  Mapper: keyframe ingestion, sensor attachment, global solve, outputs.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/interfaces/RawDataSourceBase.h>
#if __has_include(<mola_kernel/interfaces/ImuBiasSource.h>)
#include <mola_kernel/interfaces/ImuBiasSource.h>
#define MOLA_MAPPER_HAS_IMU_BIAS_SOURCE 1
#endif
#include <mola_mapper/GlobalGraphIO.h>
#include <mola_mapper/Mapper.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>

IMPLEMENTS_MRPT_OBJECT(Mapper, mola::ExecutableBase, mola::mapper)

namespace mola::mapper
{
namespace
{
double abs_time_diff(const mrpt::Clock::time_point & a, const mrpt::Clock::time_point & b)
{
  return std::abs(mrpt::system::timeDifference(a, b));
}

/** A copy of the sensory frame with `add` and without `remove`. A new one, not
 *  modified in place: snapshots of the keyframes share the old one. */
mrpt::obs::CSensoryFrame::Ptr replaced(
  const mrpt::obs::CSensoryFrame & sf, const mrpt::obs::CObservation::Ptr & remove,
  const mrpt::obs::CObservation::Ptr & add)
{
  auto out = mrpt::obs::CSensoryFrame::Create();
  for (const auto & o : sf) {
    if (o != remove) {
      out->insert(std::const_pointer_cast<mrpt::obs::CObservation>(o));
    }
  }
  if (add) {
    out->insert(add);
  }
  return out;
}

/** A copy of `img` whose image refers to `fileName` (lazy-load) instead of
 *  holding the pixels. `img` is left untouched: CImage copies are shallow, and
 *  the observation is shared with other modules. */
mrpt::obs::CObservationImage::Ptr external_image_copy(
  const mrpt::obs::CObservationImage & img, const std::string & fileName)
{
  auto copy = std::make_shared<mrpt::obs::CObservationImage>(img);
  copy->image = mrpt::img::CImage();
  copy->image.setExternalStorage(fileName);
  return copy;
}

/// Runs one end-of-run step, so a failure in it does not prevent the others.
void guarded(mrpt::system::COutputLogger & log, const char * what, const std::function<void()> & f)
{
  try {
    f();
  } catch (const std::exception & e) {
    log.logFmt(mrpt::system::LVL_ERROR, "%s failed: %s", what, e.what());
  }
}

}  // namespace

Mapper::Mapper()
{
  this->setLoggerName("Mapper");
  ExecutableBase::setModuleInstanceName("Mapper");
}

Mapper::~Mapper()
{
  try {
    finalize();
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("finalize() failed: " << e.what());
  }
  stop_loop_closure_thread();
  stop_optimizer_thread();
}

void Mapper::initialize(const mrpt::containers::yaml & cfg)
{
  MRPT_START
  ASSERTMSG_(cfg.has("params"), "The YAML configuration must have a `params` entry.");

  params_ = Parameters();
  params_.loadFrom(cfg["params"]);

  use_gnss_input_ = !params_.gnss_sensor_labels_re.empty();
  use_imu_input_ = !params_.imu_sensor_labels_re.empty();
  if (use_gnss_input_) {
    gnss_labels_re_ = std::regex(params_.gnss_sensor_labels_re);
  }
  if (use_imu_input_) {
    imu_labels_re_ = std::regex(params_.imu_sensor_labels_re);
  }
  use_image_input_ = !params_.image_sensor_labels_re.empty();
  if (use_image_input_) {
    image_labels_re_ = std::regex(params_.image_sensor_labels_re);
  }
  image_dir_.clear();
  if (use_image_input_ && params_.image_format != "inline" && !params_.output_simplemap.empty()) {
    image_dir_ = mrpt::system::fileNameChangeExtension(params_.output_simplemap, "") + "_Images";
    ASSERTMSG_(
      mrpt::system::directoryExists(image_dir_) || mrpt::system::createDirectory(image_dir_),
      "Cannot create the keyframe images directory: " + image_dir_);
  }
  use_gyro_input_ = !params_.gyro_sensor_labels_re.empty();
  if (use_gyro_input_) {
    gyro_labels_re_ = std::regex(params_.gyro_sensor_labels_re);
#if defined(MOLA_MAPPER_HAS_IMU_BIAS_SOURCE)
    imu_bias_sources_.clear();
    for (const auto & m : findService<mola::ImuBiasSource>()) {
      if (auto src = std::dynamic_pointer_cast<mola::ImuBiasSource>(m); src) {
        imu_bias_sources_.push_back(src);
      }
    }
#endif
  }
  use_wheel_odom_input_ = !params_.wheel_odom_sensor_labels_re.empty();
  if (use_wheel_odom_input_) {
    wheel_odom_labels_re_ = std::regex(params_.wheel_odom_sensor_labels_re);
  }

  // Before attaching the sources, so their first readings are recorded too:
  if (!params_.record_input.empty()) {
    recorder_ = std::make_unique<MapperInputRecorder>(params_.record_input);
    MRPT_LOG_INFO_STREAM("Recording the mapper input to: " << params_.record_input);
  }

  // Raw sensor input (GNSS, IMU), as any other RawDataConsumer module:
  if (cfg.has("raw_data_source")) {
    std::vector<std::string> sources;
    const auto ds = cfg["raw_data_source"];
    if (ds.isSequence()) {
      for (const auto & v : ds.asSequence()) {
        sources.push_back(v.as<std::string>());
      }
    } else {
      sources.push_back(ds.as<std::string>());
    }
    for (const auto & name : sources) {
      ASSERT_(this->nameServer_);
      auto src = std::dynamic_pointer_cast<RawDataSourceBase>(nameServer_(name));
      ASSERTMSG_(
        src, mrpt::format("Cannot find a raw data source module named `%s`", name.c_str()));
      src->attachToDataConsumer(*this);
    }
  }

  if (params_.enable_optimizer_thread) {
    optimizer_should_exit_.store(false);
    optimizer_thread_ = std::thread(&Mapper::optimizer_thread_loop, this);
  }
  start_loop_closure_thread();

  MRPT_LOG_INFO_STREAM(
    "Initialized: GNSS="
    << (use_gnss_input_ ? params_.gnss_sensor_labels_re : "none")
    << ", gravity IMU=" << (use_imu_input_ ? params_.imu_sensor_labels_re : "none")
    << ", gyro IMU=" << (use_gyro_input_ ? params_.gyro_sensor_labels_re : "none")
    << ", wheel odometry=" << (use_wheel_odom_input_ ? params_.wheel_odom_sensor_labels_re : "none")
    << ", images=" << (use_image_input_ ? params_.image_sensor_labels_re : "none")
    << ", loop closure=" << (params_.loop_closure_enabled ? "on" : "off")
    << ", feedback to the state estimator="
    << (params_.feedback_to_state_estimator ? "on" : "off"));
  MRPT_END
}

void Mapper::spinOnce() {}

void Mapper::onQuit() { finalize(); }

// ---------------------------------------------------------------------------
// Sensor input
// ---------------------------------------------------------------------------
void Mapper::onNewObservation(const CObservation::ConstPtr & o)
{
  if (!o) {
    return;
  }
  const auto gps = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o);
  const bool isGnss = use_gnss_input_ && gps && gps->has_GGA_datum() &&
                      std::regex_match(gps->sensorLabel, gnss_labels_re_);
  const auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
  const bool isGyro = use_gyro_input_ && imu && imu->has(mrpt::obs::IMU_WX) &&
                      imu->has(mrpt::obs::IMU_WY) && imu->has(mrpt::obs::IMU_WZ) &&
                      std::regex_match(imu->sensorLabel, gyro_labels_re_);
  const bool isGravity = use_imu_input_ && imu && imu->has(mrpt::obs::IMU_X_ACC) &&
                         std::regex_match(imu->sensorLabel, imu_labels_re_);
  const auto img = std::dynamic_pointer_cast<const mrpt::obs::CObservationImage>(o);
  const bool isImage =
    use_image_input_ && img && std::regex_match(img->sensorLabel, image_labels_re_);
  const auto odo = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o);
  const bool isWheelOdom =
    use_wheel_odom_input_ && odo && std::regex_match(odo->sensorLabel, wheel_odom_labels_re_);
  if (!isGnss && !isGyro && !isGravity && !isImage && !isWheelOdom) {
    return;
  }

  if (isImage) {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (finalized_) {
      return;
    }
    // Not recorded: images do not change the graph, and they would make the
    // record huge. Replays have no images.
    const auto im = std::const_pointer_cast<mrpt::obs::CObservationImage>(img);
    pending_images_.push_back(im);
    retry_pending_images_locked();
    finish_images_locked(std::make_pair(im->sensorLabel, im->timestamp));
    return;
  }

  // Recorded and applied in one critical section, so the recorded order is the
  // order in which the state changed:
  auto lck = mrpt::lockHelper(state_mtx_);
  if (finalized_) {
    return;
  }
  if (recorder_) {
    recorder_->record(o);
  }
  if (isGnss) {
    attach_gnss_locked(std::const_pointer_cast<mrpt::obs::CObservationGPS>(gps));
    return;
  }
  if (isGyro) {
    add_gyro_sample_locked(*imu);
  }
  if (isGravity) {
    add_gravity_sample_locked(*imu);
  }
  if (isWheelOdom) {
    add_wheel_odom_sample_locked(*odo);
  }
}

void Mapper::add_wheel_odom_sample_locked(const mrpt::obs::CObservationOdometry & odo)
{
  if (wheel_odom_label_.empty()) {
    wheel_odom_label_ = odo.sensorLabel;
  }
  if (
    odo.sensorLabel != wheel_odom_label_ ||
    (!wheel_odom_samples_.empty() && wheel_odom_samples_.back().stamp >= odo.timestamp)) {
    return;
  }
  wheel_odom_samples_.push_back({odo.timestamp, odo.odometry.asTPose()});
  while (!pending_wheel_odom_.empty() && attach_wheel_odom_locked(pending_wheel_odom_.front())) {
    pending_wheel_odom_.pop_front();
  }
  // Keep enough for keyframes arriving late, and always back to the keyframe the
  // next increment starts at (however long the vehicle stood still):
  std::optional<mrpt::Clock::time_point> needed;
  if (!pending_wheel_odom_.empty()) {
    needed =
      keyframes_.at(pending_wheel_odom_.front() > 0 ? pending_wheel_odom_.front() - 1 : 0).stamp;
  } else if (!keyframes_.empty()) {
    needed = keyframes_.back().stamp;
  }
  constexpr double WHEEL_ODOM_KEEP = 60.0;  // [s]
  while (wheel_odom_samples_.size() > 1 &&
         mrpt::system::timeDifference(wheel_odom_samples_.front().stamp, odo.timestamp) >
           WHEEL_ODOM_KEEP &&
         (!needed || wheel_odom_samples_[1].stamp <= *needed)) {
    wheel_odom_samples_.pop_front();
  }
}

bool Mapper::attach_wheel_odom_locked(std::size_t idx)
{
  if (idx == 0 || idx >= keyframes_.size()) {
    return true;
  }
  mrpt::math::TPose2D inc;
  const auto st = wheel_odometry_increment(
    wheel_odom_samples_, keyframes_[idx - 1].stamp, keyframes_[idx].stamp,
    params_.wheel_odom_max_gap, inc);
  if (st == WheelOdomStatus::NotYet) {
    return false;
  }
  if (st == WheelOdomStatus::Ok) {
    keyframes_[idx].wheel_odom_from_prev = inc;
  }
  return true;
}

void Mapper::add_gyro_sample_locked(const mrpt::obs::CObservationIMU & imu)
{
  if (gyro_label_.empty()) {
    gyro_label_ = imu.sensorLabel;
  }
  if (
    imu.sensorLabel != gyro_label_ ||
    (!gyro_samples_.empty() && gyro_samples_.back().stamp >= imu.timestamp)) {
    return;
  }
  gyro_samples_.push_back(
    {imu.timestamp,
     {imu.get(mrpt::obs::IMU_WX), imu.get(mrpt::obs::IMU_WY), imu.get(mrpt::obs::IMU_WZ)}});
  gyro_sensor_pose_ = imu.sensorPose;
  // Keep enough for keyframes arriving late:
  constexpr double GYRO_KEEP = 60.0;  // [s]
  while (mrpt::system::timeDifference(gyro_samples_.front().stamp, imu.timestamp) > GYRO_KEEP) {
    gyro_samples_.pop_front();
  }
}

void Mapper::add_gravity_sample_locked(const mrpt::obs::CObservationIMU & imu)
{
  if (gravity_label_.empty()) {
    gravity_label_ = imu.sensorLabel;
  }
  if (imu.sensorLabel != gravity_label_) {
    return;
  }
  imu_samples_.push_back(
    {imu.timestamp,
     {imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
      imu.get(mrpt::obs::IMU_Z_ACC)}});
  gravity_sensor_pose_ = imu.sensorPose;
  while (!pending_gravity_.empty() &&
         keyframes_.at(pending_gravity_.front()).stamp < imu.timestamp) {
    attach_gravity_locked(keyframes_.at(pending_gravity_.front()));
    pending_gravity_.pop_front();
  }
  // Keep a bit more than the window, in case keyframes arrive late:
  const double keep = params_.imu_gravity_window + 5.0;
  while (!imu_samples_.empty() &&
         mrpt::system::timeDifference(imu_samples_.front().stamp, imu.timestamp) > keep) {
    imu_samples_.pop_front();
  }
}

Mapper::AttachResult Mapper::attach_nearest_locked(
  const mrpt::obs::CObservation::Ptr & o, const std::string & slot, double maxOffset,
  std::size_t * attachedTo)
{
  if (keyframes_.empty() || keyframes_.back().stamp < o->timestamp) {
    return AttachResult::Pending;
  }
  const auto it = std::lower_bound(
    keyframes_.begin(), keyframes_.end(), o->timestamp,
    [](const Keyframe & kf, const mrpt::Clock::time_point & t) { return kf.stamp < t; });

  Keyframe * best = nullptr;
  double bestDt = maxOffset;
  for (auto c : {it, it == keyframes_.begin() ? keyframes_.end() : std::prev(it)}) {
    if (c == keyframes_.end()) {
      continue;
    }
    const double dt = abs_time_diff(c->stamp, o->timestamp);
    if (dt <= bestDt) {
      bestDt = dt;
      best = &*c;
    }
  }
  if (best == nullptr) {
    return AttachResult::Dropped;  // no keyframe close enough in time
  }
  mrpt::obs::CObservation::Ptr prev;
  if (const auto n = best->nearest.find(slot); n != best->nearest.end()) {
    if (n->second.dt <= bestDt) {
      return AttachResult::Dropped;  // that keyframe already has a closer one
    }
    prev = n->second.obs;
  }
  best->aux = replaced(*best->aux, prev, o);
  best->nearest[slot] = {o, bestDt};
  if (attachedTo != nullptr) {
    *attachedTo = static_cast<std::size_t>(best - keyframes_.data());
  }
  return AttachResult::Attached;
}

void Mapper::attach_gnss_locked(const mrpt::obs::CObservationGPS::Ptr & gps)
{
  switch (attach_nearest_locked(gps, "gnss", params_.gnss_max_time_offset)) {
    case AttachResult::Pending:
      pending_gnss_.push_back(gps);
      break;
    case AttachResult::Attached:
      optimizer_new_data_.store(true);
      break;
    case AttachResult::Dropped:
      break;
  }
}

bool Mapper::attach_or_pend_image_locked(const mrpt::obs::CObservationImage::Ptr & img)
{
  const std::string slot = "image:" + img->sensorLabel;
  std::size_t kfIdx = 0;
  switch (attach_nearest_locked(img, slot, params_.image_max_time_offset, &kfIdx)) {
    case AttachResult::Pending:
      return false;
    case AttachResult::Attached: {
      const std::pair<std::size_t, std::string> entry{kfIdx, slot};
      if (
        std::find(unwritten_images_.begin(), unwritten_images_.end(), entry) ==
        unwritten_images_.end()) {
        unwritten_images_.push_back(entry);
      }
      break;
    }
    case AttachResult::Dropped:
      break;
  }
  return true;
}

void Mapper::retry_pending_images_locked()
{
  const std::deque<mrpt::obs::CObservationImage::Ptr> pending = std::move(pending_images_);
  pending_images_.clear();
  for (const auto & img : pending) {
    if (!attach_or_pend_image_locked(img)) {
      pending_images_.push_back(img);
    }
  }
  // Bounded, in case keyframes stop arriving (e.g. a stopped vehicle): older
  // ones could only go to a keyframe farther than image_max_time_offset from
  // them, but for the front end lag.
  constexpr double FRONT_END_LAG_MARGIN = 2.0;  // [s]
  while (!pending_images_.empty() &&
         mrpt::system::timeDifference(
           pending_images_.front()->timestamp, pending_images_.back()->timestamp) >
           params_.image_max_time_offset + FRONT_END_LAG_MARGIN) {
    pending_images_.pop_front();
  }
}

void Mapper::finish_images_locked(
  const std::optional<std::pair<std::string, mrpt::Clock::time_point>> & newest)
{
  for (auto it = unwritten_images_.begin(); it != unwritten_images_.end();) {
    auto & kf = keyframes_.at(it->first);
    auto & n = kf.nearest.at(it->second);
    // Images of a camera come in time order: a newer one, farther from the
    // keyframe than the attached one, means that no closer one can come any
    // more. Except for the newest keyframe: pending images may still go to it.
    if (
      newest && (it->first + 1 == keyframes_.size() || it->second != "image:" + newest->first ||
                 abs_time_diff(kf.stamp, newest->second) <= n.dt || newest->second < kf.stamp)) {
      ++it;
      continue;
    }
    const auto img = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(n.obs);
    ASSERT_(img);
    // Already a file (e.g. from a lazy-load dataset), or not 8-bit: kept as is.
    if (
      !image_dir_.empty() && !img->image.isExternallyStored() &&
      img->image.getPixelDepth() == mrpt::img::PixelDepth::D8U) {
      const std::string fileName = mrpt::format(
        "%s_%.09f.%s", mrpt::system::fileNameStripInvalidChars(img->sensorLabel).c_str(),
        mrpt::Clock::toDouble(img->timestamp), params_.image_format.c_str());
      auto ext = external_image_copy(*img, fileName);
      kf.aux = replaced(*kf.aux, n.obs, ext);
      n.obs = ext;
      const auto path = mrpt::system::pathJoin({image_dir_, fileName});
      const int quality = params_.image_jpeg_quality;
      image_writes_.push_back(disk_io_.enqueue([this, img, path, quality]() {
        try {
          if (!img->image.saveToFile(path, quality)) {
            MRPT_LOG_ERROR_STREAM("Could not write the keyframe image: " << path);
          }
        } catch (const std::exception & e) {
          MRPT_LOG_ERROR_STREAM("Could not write the keyframe image: " << path << ": " << e.what());
        }
      }));
    }
    it = unwritten_images_.erase(it);
  }
  // Forget the writes already done:
  while (!image_writes_.empty() &&
         image_writes_.front().wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    image_writes_.pop_front();
  }
}

void Mapper::retry_pending_gnss_locked()
{
  const std::deque<mrpt::obs::CObservationGPS::Ptr> pending = std::move(pending_gnss_);
  pending_gnss_.clear();
  for (const auto & gps : pending) {
    attach_gnss_locked(gps);
  }
}

Mapper::GyroBias Mapper::query_gyro_bias(
  [[maybe_unused]] const std::string & label,
  [[maybe_unused]] const mrpt::Clock::time_point & t) const
{
  // From a bias source, if any has an estimate for this IMU:
  GyroBias b;
  b.sigma = params_.gyro_bias_sigma;
#if defined(MOLA_MAPPER_HAS_IMU_BIAS_SOURCE)
  if (label.empty()) {
    return b;
  }
  for (const auto & src : imu_bias_sources_) {
    if (const auto e = src->imu_bias(label, t); e && e->last_update) {
      b.bias = e->gyro;
      b.sigma = std::sqrt(std::max({e->gyro_cov(0, 0), e->gyro_cov(1, 1), e->gyro_cov(2, 2)}));
      b.from_source = true;
      break;
    }
  }
#endif
  return b;
}

void Mapper::attach_gyro_rotation_locked(Keyframe & kf, const GyroBias & b)
{
  if (!use_gyro_input_ || keyframes_.empty() || gyro_samples_.empty()) {
    return;
  }
  const auto & prev = keyframes_.back();
  const auto g = integrate_gyro(gyro_samples_, prev.stamp, kf.stamp, b.bias, params_.gyro_max_gap);
  if (!g) {
    return;
  }
  // From the sensor to the vehicle frame: R_v = R_vs * R_s * R_vs^T.
  const auto Rvs = gyro_sensor_pose_.getRotationMatrix();
  const mrpt::math::CMatrixDouble33 Rv(Rvs.asEigen() * g->R.asEigen() * Rvs.asEigen().transpose());
  mrpt::poses::CPose3D rot;
  rot.setRotationMatrix(Rv);
  kf.gyro_rotation_from_prev = rot;
  kf.gyro_rotation_sigma = std::sqrt(
    mrpt::square(params_.gyro_noise_density) * g->duration + mrpt::square(b.sigma * g->duration) +
    mrpt::square(params_.gyro_scale_sigma * g->angle));
}

bool Mapper::attach_gravity_locked(Keyframe & kf)
{
  if (gravity_label_.empty()) {
    return false;
  }
  mrpt::math::TVector3D sum(0, 0, 0);
  uint32_t n = 0;
  for (const auto & s : imu_samples_) {
    const double dt = mrpt::system::timeDifference(s.stamp, kf.stamp);
    if (dt >= 0 && dt <= params_.imu_gravity_window) {
      sum += s.acc;
      n++;
    }
  }
  if (n < params_.imu_gravity_min_samples) {
    return false;
  }
  auto obs = mrpt::obs::CObservationIMU::Create();
  obs->sensorLabel = gravity_label_;
  obs->sensorPose = gravity_sensor_pose_;
  obs->timestamp = kf.stamp;
  obs->set(mrpt::obs::IMU_X_ACC, sum.x / n);
  obs->set(mrpt::obs::IMU_Y_ACC, sum.y / n);
  obs->set(mrpt::obs::IMU_Z_ACC, sum.z / n);
  kf.aux = replaced(*kf.aux, nullptr, obs);
  return true;
}

// ---------------------------------------------------------------------------
// Keyframes from the front end
// ---------------------------------------------------------------------------
std::optional<SharedKeyframeMap::KeyFrameID> Mapper::requestInsertKeyframe(
  const KeyframeInsertRequest & req)
{
  ASSERTMSG_(!req.source_frame_id.empty(), "KeyframeInsertRequest::source_frame_id is empty.");

  // Other modules are queried without state_mtx_ held:
  RecordedGyroBias gyroBias;
  gyroBias.sigma = params_.gyro_bias_sigma;
  if (use_gyro_input_) {
    std::string label;
    {
      auto lck = mrpt::lockHelper(state_mtx_);
      label = gyro_label_;
    }
    const auto b = query_gyro_bias(label, req.timestamp);
    gyroBias.bias = b.bias;
    gyroBias.sigma = b.sigma;
    gyroBias.from_source = b.from_source;
  }
  // Encoded here (it copies the scans), recorded with the state change below:
  const auto entry =
    recorder_ ? MapperInputRecorder::encode(RecordedKeyframe{req, gyroBias}) : nullptr;
  return insert_keyframe_impl(req, gyroBias, entry);
}

std::optional<SharedKeyframeMap::KeyFrameID> Mapper::insert_keyframe(
  const KeyframeInsertRequest & req, const RecordedGyroBias & bias)
{
  return insert_keyframe_impl(req, bias, nullptr);
}

std::optional<SharedKeyframeMap::KeyFrameID> Mapper::insert_keyframe_impl(
  const KeyframeInsertRequest & req, const RecordedGyroBias & bias,
  const mrpt::serialization::CSerializable::Ptr & recordEntry)
{
  ASSERTMSG_(!req.source_frame_id.empty(), "KeyframeInsertRequest::source_frame_id is empty.");
  // Without a bias source, the configured sigma (e.g. a replay with another one):
  const GyroBias gyroBias = bias.from_source ? GyroBias{bias.bias, bias.sigma, true}
                                             : GyroBias{{0, 0, 0}, params_.gyro_bias_sigma, false};

  KeyFrameID id = 0;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (finalized_) {
      return std::nullopt;
    }
    if (recordEntry && recorder_) {
      recorder_->record(recordEntry);
    }
    if (!keyframes_.empty() && req.timestamp <= keyframes_.back().stamp) {
      MRPT_LOG_WARN_STREAM(
        "Ignoring a keyframe older than the newest one: "
        << mrpt::system::dateTimeToString(req.timestamp));
      return std::nullopt;
    }

    Keyframe kf;
    kf.stamp = req.timestamp;
    kf.source_frame = req.source_frame_id;
    kf.pose_in_source = req.pose_in_source;
    kf.observations = req.observations;

    // Initial estimate: the newest keyframe estimate plus the front end increment
    // (the first keyframe defines the map frame):
    if (keyframes_.empty()) {
      kf.pose_in_map = req.pose_in_source.mean;
    } else if (keyframes_.back().source_frame == kf.source_frame) {
      const auto & prev = keyframes_.back();
      kf.pose_in_map = prev.pose_in_map + (kf.pose_in_source.mean - prev.pose_in_source.mean);
    } else {
      kf.pose_in_map = keyframes_.back().pose_in_map;
    }

    // If the IMU readings up to the keyframe have not all arrived yet, tried
    // again once they have:
    const bool imuBehind =
      use_imu_input_ && (imu_samples_.empty() || imu_samples_.back().stamp < kf.stamp);
    const bool gravityAttached = attach_gravity_locked(kf);
    attach_gyro_rotation_locked(kf, gyroBias);
    keyframes_.push_back(std::move(kf));
    id = keyframes_.size() - 1;
    if (!gravityAttached && imuBehind) {
      pending_gravity_.push_back(id);
    }
    // In order: a keyframe waits if older ones do.
    if (use_wheel_odom_input_ && (!pending_wheel_odom_.empty() || !attach_wheel_odom_locked(id))) {
      pending_wheel_odom_.push_back(id);
      constexpr std::size_t MANY_PENDING = 20;
      if (pending_wheel_odom_.size() > MANY_PENDING) {
        const std::string why = wheel_odom_samples_.empty()
                                  ? "none received yet with a label matching '" +
                                      params_.wheel_odom_sensor_labels_re + "'"
                                  : "the newest one is older than the keyframes";
        MRPT_LOG_THROTTLE_WARN_FMT(
          30.0, "%zu keyframes waiting for wheel odometry readings: %s", pending_wheel_odom_.size(),
          why.c_str());
      }
    }

    retry_pending_gnss_locked();
    retry_pending_images_locked();
  }
  optimizer_new_data_.store(true);
  notify_optimizer();
  return id;
}

// ---------------------------------------------------------------------------
// Global graph
// ---------------------------------------------------------------------------
void Mapper::notify_optimizer()
{
  {
    auto lck = mrpt::lockHelper(optimizer_wakeup_mtx_);
  }
  optimizer_wakeup_cv_.notify_one();
}

void Mapper::optimizer_thread_loop()
{
  const auto minPeriod = std::chrono::duration<double>(std::max(0.0, params_.optimizer_min_period));
  while (!optimizer_should_exit_.load()) {
    {
      std::unique_lock<std::mutex> lk(optimizer_wakeup_mtx_);
      optimizer_wakeup_cv_.wait_for(lk, std::chrono::seconds(1), [this] {
        return optimizer_should_exit_.load() || optimizer_new_data_.load();
      });
    }
    if (optimizer_should_exit_.load()) {
      break;
    }
    if (!optimizer_new_data_.exchange(false)) {
      continue;
    }
    try {
      solve_global_graph();
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Global graph solve failed: " << e.what());
    }
    // At most one solve per period:
    std::unique_lock<std::mutex> lk(optimizer_wakeup_mtx_);
    optimizer_wakeup_cv_.wait_for(lk, minPeriod, [this] { return optimizer_should_exit_.load(); });
  }
}

void Mapper::stop_optimizer_thread()
{
  optimizer_should_exit_.store(true);
  notify_optimizer();
  if (optimizer_thread_.joinable()) {
    optimizer_thread_.join();
  }
}

void Mapper::optimize_now()
{
  optimizer_new_data_.store(false);
  solve_global_graph();
}

GlobalGraphInput Mapper::graph_input_locked() const
{
  GlobalGraphInput in;
  in.keyframes.reserve(keyframes_.size());
  for (const auto & kf : keyframes_) {
    GlobalGraphKeyframe g;
    g.stamp = kf.stamp;
    g.source_frame = kf.source_frame;
    g.pose_in_source = kf.pose_in_source;
    g.initial_guess = kf.pose_in_map;
    g.gyro_rotation_from_prev = kf.gyro_rotation_from_prev;
    g.gyro_rotation_sigma = kf.gyro_rotation_sigma;
    g.wheel_odom_from_prev = kf.wheel_odom_from_prev;
    // A copy: the original may get new GNSS readings while solving. Without
    // images, which the graph does not use:
    auto aux = mrpt::obs::CSensoryFrame::Create();
    for (const auto & o : *kf.aux) {
      if (!std::dynamic_pointer_cast<const mrpt::obs::CObservationImage>(o)) {
        aux->insert(std::const_pointer_cast<mrpt::obs::CObservation>(o));
      }
    }
    g.aux_observations = aux;
    in.keyframes.push_back(std::move(g));
  }
  in.loops = loops_;
  in.geodetic_datum = geodetic_datum_;
  if (georef_) {
    in.T_enu_to_map_guess = georef_->T_enu_to_map.mean;
  }
  return in;
}

void Mapper::solve_global_graph()
{
  auto lckSolve = mrpt::lockHelper(solve_mtx_);

  GlobalGraphInput in;
  std::size_t loopsRevision = 0;
  std::size_t loopsAdded = 0;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (keyframes_.empty()) {
      return;
    }
    in = graph_input_locked();
    loopsRevision = loops_revision_;
    loopsAdded = loops_added_;
  }

  const auto t0 = std::chrono::steady_clock::now();
  GlobalGraphResult r = optimize_global_graph(in, params_.graph, this);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  mrpt::Clock::time_point newestStamp;
  mrpt::poses::CPose3D newestPose;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    const std::size_t n = in.keyframes.size();
    for (std::size_t i = 0; i < n; i++) {
      keyframes_[i].pose_in_map = r.poses[i];
    }
    // Keyframes inserted while solving: re-chained onto the new estimates.
    for (std::size_t i = n; i < keyframes_.size(); i++) {
      auto & kf = keyframes_[i];
      const auto & prev = keyframes_[i - 1];
      if (prev.source_frame == kf.source_frame) {
        kf.pose_in_map = prev.pose_in_map + (kf.pose_in_source.mean - prev.pose_in_source.mean);
      } else {
        kf.pose_in_map = prev.pose_in_map;
      }
    }
    if (r.geodetic_datum) {
      geodetic_datum_ = r.geodetic_datum;
    }
    if (r.georeferencing) {
      georef_ = r.georeferencing;
    }
    newestStamp = keyframes_[n - 1].stamp;
    newestPose = keyframes_[n - 1].pose_in_map;
    last_num_gnss_factors_ = r.num_gnss_factors;
    last_num_gravity_factors_ = r.num_gravity_factors;
    last_num_gyro_factors_ = r.num_gyro_factors;
    last_num_wheel_odom_factors_ = r.num_wheel_odom_factors;
  }
  solves_++;

  MRPT_LOG_THROTTLE_INFO_FMT(
    10.0,
    "Global graph: %zu keyframes, %zu loops, %zu GNSS%s, %zu gravity, %zu gyro, %zu wheel "
    "odometry factors, solved in %.0f ms",
    in.keyframes.size(), r.num_loop_factors, r.num_gnss_factors,
    r.gnss_degenerate ? " (degenerate, unused)" : "", r.num_gravity_factors, r.num_gyro_factors,
    r.num_wheel_odom_factors, 1e3 * dt);

  if (r.georeferencing) {
    publish_georeference(*r.georeferencing);
  }
  send_feedback(newestStamp, newestPose, r);

#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
  // Only loop closures (added or dropped) move the keyframes around the front
  // end enough to rebuild its local map:
  if (params_.local_map_corrections && loopsRevision != loops_revision_in_last_correction_) {
    mola::LocalMapCorrection c;
    c.timestamp = newestStamp;
    c.new_loop_closures = loopsAdded - loops_added_in_last_correction_;
    loops_revision_in_last_correction_ = loopsRevision;
    loops_added_in_last_correction_ = loopsAdded;
    advertise_local_map_correction(c);
  }
#endif
}

#if defined(MOLA_MAPPER_HAS_LOCAL_MAP_CORRECTIONS)
std::optional<std::vector<mola::LocalMapKeyframe>> Mapper::local_map_keyframes_impl(
  KeyFrameID reference, double radius) const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  if (reference >= keyframes_.size()) {
    return std::nullopt;
  }
  const auto & ref = keyframes_.at(reference).pose_in_map;
  std::vector<mola::LocalMapKeyframe> out;
  for (std::size_t i = 0; i < keyframes_.size(); i++) {
    const auto & kf = keyframes_[i];
    if ((kf.pose_in_map.translation() - ref.translation()).norm() > radius) {
      continue;
    }
    mola::LocalMapKeyframe k;
    k.id = i;
    k.timestamp = kf.stamp;
    k.source_frame_id = kf.source_frame;
    k.pose_from_reference = kf.pose_in_map - ref;
    k.observations = kf.observations;
    out.push_back(std::move(k));
  }
  return out;
}
#endif

void Mapper::send_feedback(
  const mrpt::Clock::time_point & stamp, const mrpt::poses::CPose3D & pose,
  const GlobalGraphResult & r)
{
  if (!params_.feedback_to_state_estimator) {
    return;
  }
  // Each keyframe is fed back once: solves without a new keyframe (loops, late
  // GNSS) would otherwise count the same information again.
  if (last_feedback_stamp_ && *last_feedback_stamp_ >= stamp) {
    return;
  }
  auto se = state_estimator_.lock();
  if (!se) {
    const auto mods = findService<mola::NavStateFilter>();
    if (mods.size() != 1) {
      MRPT_LOG_THROTTLE_WARN(
        30.0, "feedback_to_state_estimator: there must be exactly one NavStateFilter module");
      return;
    }
    se = std::dynamic_pointer_cast<mola::NavStateFilter>(mods[0]);
    state_estimator_ = se;
  }

  mrpt::poses::CPose3DPDFGaussian pdf;
  pdf.mean = pose;
  if (r.last_pose) {
    pdf.cov = r.last_pose->cov;
  }
  const double sXyz2 = mrpt::square(params_.feedback_min_sigma_xyz);
  const double sAng2 = mrpt::square(mrpt::DEG2RAD(params_.feedback_min_sigma_ang_deg));
  for (int i = 0; i < 6; i++) {
    pdf.cov(i, i) = std::max(pdf.cov(i, i), i < 3 ? sXyz2 : sAng2);
  }
  se->fuse_pose(stamp, pdf, params_.reference_frame_name);
  last_feedback_stamp_ = stamp;
}

void Mapper::publish_georeference(const mola::Georeferencing & g)
{
  MapUpdate mu;
  mu.timestamp = mrpt::Clock::now();
  mu.reference_frame = params_.reference_frame_name;
  mu.method = "mapper";
  mu.map_name = "georeference";
  mu.georeferencing = g;
  advertiseUpdatedMap(mu);
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------
std::size_t Mapper::keyframe_count() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  return keyframes_.size();
}

std::vector<mrpt::poses::CPose3D> Mapper::keyframe_poses() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  std::vector<mrpt::poses::CPose3D> out;
  out.reserve(keyframes_.size());
  for (const auto & kf : keyframes_) {
    out.push_back(kf.pose_in_map);
  }
  return out;
}

std::size_t Mapper::loop_closure_count() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  return loops_.size();
}

std::vector<std::pair<std::size_t, std::size_t>> Mapper::loop_closure_pairs() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  std::vector<std::pair<std::size_t, std::size_t>> out;
  out.reserve(loops_.size());
  for (const auto & l : loops_) {
    out.emplace_back(std::min(l.from, l.to), std::max(l.from, l.to));
  }
  return out;
}

std::optional<mola::Georeferencing> Mapper::georeferencing() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  return georef_;
}

mrpt::maps::CSimpleMap Mapper::simple_map() const
{
  auto lck = mrpt::lockHelper(state_mtx_);
  return to_simple_map(keyframes_);
}

mrpt::maps::CSimpleMap Mapper::to_simple_map(const std::vector<Keyframe> & kfs)
{
  mrpt::maps::CSimpleMap sm;
  for (const auto & kf : kfs) {
    auto sf = std::make_shared<mrpt::obs::CSensoryFrame>(kf.observations);
    for (const auto & o : *kf.aux) {
      sf->insert(o);
    }
    sm.insert(mrpt::poses::CPose3DPDFGaussian::Create(kf.pose_in_map), sf);
  }
  return sm;
}

void Mapper::getDiagnostics(std::vector<mola::DiagnosticStatusMsg> & status)
{
  mola::DiagnosticStatusMsg s;
  s.name = "Mapper";
  s.hardware_id = "mola_mapper";
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    s.values.push_back({"keyframes", std::to_string(keyframes_.size())});
    s.values.push_back({"loop_closures", std::to_string(loops_.size())});
    s.values.push_back({"gnss_factors", std::to_string(last_num_gnss_factors_)});
    s.values.push_back({"gravity_factors", std::to_string(last_num_gravity_factors_)});
    s.values.push_back({"gyro_factors", std::to_string(last_num_gyro_factors_)});
    s.values.push_back({"wheel_odom_factors", std::to_string(last_num_wheel_odom_factors_)});
    s.values.push_back({"geo_referenced", georef_ ? "yes" : "no"});
    s.message = mrpt::format(
      "%zu keyframes, %zu loop closures%s", keyframes_.size(), loops_.size(),
      georef_ ? ", geo-referenced" : "");
  }
  s.values.push_back({"solves", std::to_string(solves_.load())});
  s.values.push_back({"lc_scans", std::to_string(lc_scans_.load())});
  s.values.push_back({"lc_candidates_checked", std::to_string(lc_candidates_checked_.load())});
  s.values.push_back({"lc_rejected_by_odometry", std::to_string(lc_rejected_by_odometry_.load())});
  s.values.push_back(
    {"lc_currently_rejected_by_consistency",
     std::to_string(lc_currently_rejected_by_consistency_.load())});
  s.values.push_back({"lc_scan_in_progress", lc_busy_.load() ? "yes" : "no"});
  status.push_back(std::move(s));
}

// ---------------------------------------------------------------------------
// End of run
// ---------------------------------------------------------------------------
void Mapper::finalize()
{
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    if (finalized_) {
      return;
    }
    finalized_ = true;
  }
  if (recorder_) {
    // No more input to record: the end of the run.
    recorder_->close();
    MRPT_LOG_INFO_STREAM(
      "Mapper input recorded: " << recorder_->written() << " entries in " << params_.record_input);
  }
  stop_loop_closure_thread();
  stop_optimizer_thread();
  // The final scans below run in this thread and must not be aborted:
  lc_should_exit_.store(false);

  // Each step is guarded, so the outputs are written whatever fails before them:
  guarded(*this, "Final solve", [this] { optimize_now(); });
  if (lc_engine_) {
    // Each correction may bring further loops within reach of the detector:
    for (uint32_t round = 0; round < params_.loop_closure_finalize_rounds; round++) {
      const auto loopsRevision = [this] {
        auto lck = mrpt::lockHelper(state_mtx_);
        return loops_revision_;
      };
      const std::size_t revisionBefore = loopsRevision();
      std::size_t n = 0;
      guarded(*this, "Final loop closure pass", [&] { n = run_loop_closure_scan(true); });
      MRPT_LOG_INFO_STREAM("Final loop closure pass #" << round + 1 << ": " << n << " new loops");
      // Solved again whenever the loops changed (also if some were only dropped):
      if (loopsRevision() == revisionBefore) {
        break;
      }
      guarded(*this, "Final solve", [this] { optimize_now(); });
    }
  }
  // The images still waiting for a possibly closer one, and all the image
  // files written before the simplemap that refers to them:
  guarded(*this, "Writing the keyframe images", [this] {
    std::deque<std::future<void>> writes;
    {
      auto lck = mrpt::lockHelper(state_mtx_);
      finish_images_locked(std::nullopt);
      writes = std::move(image_writes_);
      image_writes_.clear();
    }
    for (auto & w : writes) {
      w.wait();
    }
  });
  save_outputs();
}

void Mapper::save_outputs()
{
  // A single snapshot, so all the outputs agree:
  std::vector<Keyframe> kfs;
  std::vector<GlobalGraphLoopEdge> loops;
  std::optional<mola::Georeferencing> georef;
  GlobalGraphInput in;
  {
    auto lck = mrpt::lockHelper(state_mtx_);
    kfs = keyframes_;
    loops = loops_;
    georef = georef_;
    if (!params_.output_graph_input.empty()) {
      in = graph_input_locked();
    }
  }

  if (!params_.output_graph_input.empty()) {
    guarded(*this, "Saving the global graph input", [&] {
      save_global_graph_input(in, params_.output_graph_input);
      MRPT_LOG_INFO_STREAM(
        "Saved the global graph input to " << params_.output_graph_input << ".*");
    });
  }

  if (!params_.output_tum_trajectory.empty()) {
    guarded(*this, "Saving the TUM trajectory", [&] {
      std::vector<std::pair<mrpt::Clock::time_point, mrpt::poses::CPose3D>> poses;
      for (const auto & kf : kfs) {
        poses.emplace_back(kf.stamp, kf.pose_in_map);
      }
      save_tum_trajectory(poses, params_.output_tum_trajectory);
      MRPT_LOG_INFO_STREAM(
        "Saved " << kfs.size() << " keyframe poses to " << params_.output_tum_trajectory);
    });
  }
  if (!params_.output_georeference.empty() && georef) {
    guarded(*this, "Saving the geo-reference", [&] {
      save_georeference(*georef, params_.output_georeference);
      MRPT_LOG_INFO_STREAM("Saved the geo-reference to " << params_.output_georeference);
    });
  }

  // The state estimator view, for comparisons:
  const bool wantSe = !params_.output_state_estimator_tum.empty() ||
                      !params_.output_state_estimator_georeference.empty();
  if (wantSe && !kfs.empty()) {
    guarded(
      *this, "Saving the state estimator outputs", [&] { save_state_estimator_outputs(kfs); });
  }

  if (!params_.output_loop_closures.empty()) {
    guarded(*this, "Saving the loop closures", [&] {
      std::ofstream f(params_.output_loop_closures);
      ASSERTMSG_(f.is_open(), "Cannot write to " + params_.output_loop_closures);
      f << "# kf_from kf_to x y z yaw pitch roll  (pose of kf_to seen from kf_from) t_from t_to\n";
      f << std::fixed << std::setprecision(6);
      for (const auto & lc : loops) {
        const auto & p = lc.relative_pose.mean;
        f << lc.from << " " << lc.to << " " << p.x() << " " << p.y() << " " << p.z() << " "
          << p.yaw() << " " << p.pitch() << " " << p.roll() << " "
          << mrpt::Clock::toDouble(kfs.at(lc.from).stamp) << " "
          << mrpt::Clock::toDouble(kfs.at(lc.to).stamp) << "\n";
      }
    });
  }

  if (!params_.output_simplemap.empty()) {
    guarded(*this, "Saving the simplemap", [&] {
      const auto sm = to_simple_map(kfs);
      if (sm.saveToFile(params_.output_simplemap)) {
        MRPT_LOG_INFO_STREAM(
          "Saved the simplemap (" << sm.size() << " keyframes) to " << params_.output_simplemap);
      } else {
        MRPT_LOG_ERROR_STREAM("Could not save the simplemap to " << params_.output_simplemap);
      }
    });
  }
}

void Mapper::save_state_estimator_outputs(const std::vector<Keyframe> & kfs)
{
  const auto mods = findService<mola::NavStateFilter>();
  auto se = mods.size() == 1 ? std::dynamic_pointer_cast<mola::NavStateFilter>(mods[0]) : nullptr;
  if (!se) {
    MRPT_LOG_WARN("No (single) NavStateFilter module: its outputs are not saved");
    return;
  }
  if (!params_.output_state_estimator_tum.empty()) {
    const auto traj = se->estimated_trajectory(
      kfs.front().stamp - std::chrono::seconds(10), kfs.back().stamp + std::chrono::seconds(10),
      params_.reference_frame_name);
    if (traj && !traj->empty()) {
      std::vector<std::pair<mrpt::Clock::time_point, mrpt::poses::CPose3D>> poses;
      for (const auto & [t, p] : *traj) {
        poses.emplace_back(t, mrpt::poses::CPose3D(p));
      }
      save_tum_trajectory(poses, params_.output_state_estimator_tum);
      MRPT_LOG_INFO_STREAM(
        "Saved " << poses.size() << " state estimator poses to "
                 << params_.output_state_estimator_tum);
    } else {
      MRPT_LOG_WARN(
        "The state estimator returned no trajectory (smoother: keep_finalized_trajectory?)");
    }
  }
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
  if (!params_.output_state_estimator_georeference.empty()) {
    if (const auto g = se->get_geo_reference(); g) {
      save_georeference(*g, params_.output_state_estimator_georeference);
    }
  }
#endif
}

}  // namespace mola::mapper
