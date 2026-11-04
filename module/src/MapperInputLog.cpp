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
 * @file   MapperInputLog.cpp
 * @brief  Record and read back everything a Mapper receives.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/MapperInputLog.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <array>
#include <charconv>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace mola::mapper
{
namespace
{
const char * KEYFRAME_LABEL = "mapper_keyframe_request";

/** Doubles in hexadecimal form (locale independent): they read back exactly. */
void write_numbers(std::string & s, const std::vector<double> & v)
{
  for (const double x : v) {
    std::array<char, 64> buf{};
    const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), x, std::chars_format::hex);
    ASSERT_(r.ec == std::errc());
    s += ' ';
    s.append(buf.data(), r.ptr);
  }
  s += '\n';
}

std::vector<double> read_numbers(const std::string & rest)
{
  std::vector<double> v;
  std::istringstream s(rest);
  std::string tok;
  while (s >> tok) {
    double x = 0;
    const auto r = std::from_chars(tok.data(), tok.data() + tok.size(), x, std::chars_format::hex);
    ASSERTMSG_(
      r.ec == std::errc() && r.ptr == tok.data() + tok.size(),
      "Bad number in the mapper input log: " + tok);
    v.push_back(x);
  }
  return v;
}

/** An observation as stored in the log: point clouds deep-copied (their owner
 *  may unload or modify them later) and embedded, even if the original is
 *  stored in an external file. */
mrpt::obs::CObservation::Ptr self_contained(const mrpt::obs::CObservation::Ptr & o)
{
  const auto pc = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(o);
  if (!pc) {
    return o;
  }
  pc->load();
  auto copy = mrpt::obs::CObservationPointCloud::Create();
  copy->sensorLabel = pc->sensorLabel;
  copy->timestamp = pc->timestamp;
  copy->sensorPose = pc->sensorPose;
  if (pc->pointcloud) {
    copy->pointcloud =
      std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(pc->pointcloud->duplicateGetSmartPtr());
  }
  return copy;
}

void decode_line(const std::string & key, const std::string & rest, RecordedKeyframe & kf)
{
  auto & r = kf.request;
  if (key == "source_frame_id") {
    r.source_frame_id = rest;
    return;
  }
  const auto v = read_numbers(rest);
  if (key == "quality") {
    ASSERT_EQUAL_(v.size(), 1U);
    r.quality = v[0];
  } else if (key == "pose") {
    ASSERT_EQUAL_(v.size(), 12U);
    mrpt::math::CMatrixDouble44 h = mrpt::math::CMatrixDouble44::Identity();
    for (int k = 0; k < 12; k++) {
      h(k / 4, k % 4) = v[k];
    }
    r.pose_in_source.mean = mrpt::poses::CPose3D(h);
  } else if (key == "cov") {
    ASSERT_EQUAL_(v.size(), 36U);
    for (int k = 0; k < 36; k++) {
      r.pose_in_source.cov(k / 6, k % 6) = v[k];
    }
  } else if (key == "gyro_bias") {
    ASSERT_EQUAL_(v.size(), 5U);
    kf.gyro_bias.bias = {v[0], v[1], v[2]};
    kf.gyro_bias.sigma = v[3];
    kf.gyro_bias.from_source = v[4] != 0;
  }
}

RecordedKeyframe decode(const mrpt::obs::CSensoryFrame & sf)
{
  RecordedKeyframe kf;
  std::set<std::string> keys;
  for (std::size_t i = 0; i < sf.size(); i++) {
    const auto o = sf.getObservationByIndex(i);
    const auto c = std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o);
    if (!c || c->sensorLabel != KEYFRAME_LABEL) {
      kf.request.observations.insert(std::const_pointer_cast<mrpt::obs::CObservation>(o));
      continue;
    }
    kf.request.timestamp = c->timestamp;
    std::istringstream lines(c->text);
    std::string line;
    while (std::getline(lines, line)) {
      const auto sp = line.find(' ');
      const std::string key = line.substr(0, sp);
      decode_line(key, sp == std::string::npos ? "" : line.substr(sp + 1), kf);
      keys.insert(key);
    }
  }
  ASSERTMSG_(
    keys == std::set<std::string>({"source_frame_id", "quality", "pose", "cov", "gyro_bias"}),
    "Incomplete keyframe request in the mapper input log.");
  return kf;
}

}  // namespace

mrpt::serialization::CSerializable::Ptr MapperInputRecorder::encode(const RecordedKeyframe & kf)
{
  const auto & r = kf.request;
  std::string s = "source_frame_id " + r.source_frame_id + "\n";
  s += "quality";
  write_numbers(s, {r.quality});
  // The 3x4 matrix, not angles, so the rotation reads back exactly:
  const auto h = r.pose_in_source.mean.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>();
  std::vector<double> pose;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      pose.push_back(h(i, j));
    }
  }
  s += "pose";
  write_numbers(s, pose);
  std::vector<double> cov;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov.push_back(r.pose_in_source.cov(i, j));
    }
  }
  s += "cov";
  write_numbers(s, cov);
  const auto & b = kf.gyro_bias;
  s += "gyro_bias";
  write_numbers(s, {b.bias.x, b.bias.y, b.bias.z, b.sigma, b.from_source ? 1.0 : 0.0});

  auto c = mrpt::obs::CObservationComment::Create();
  c->sensorLabel = KEYFRAME_LABEL;
  c->timestamp = r.timestamp;
  c->text = s;

  auto sf = mrpt::obs::CSensoryFrame::Create();
  for (std::size_t i = 0; i < r.observations.size(); i++) {
    sf->insert(self_contained(
      std::const_pointer_cast<mrpt::obs::CObservation>(r.observations.getObservationByIndex(i))));
  }
  sf->insert(c);
  return sf;
}

MapperInputRecorder::MapperInputRecorder(const std::string & file)
: out_(std::make_unique<mrpt::io::CFileGZOutputStream>())
{
  // Fast compression: the scans dominate, and writing must keep up with a live run.
  ASSERTMSG_(out_->open(file, 1), "Cannot write the mapper input log: " + file);
  writer_ = std::thread([this]() { writer_loop(); });
}

MapperInputRecorder::~MapperInputRecorder() { close(); }

void MapperInputRecorder::record(const mrpt::serialization::CSerializable::Ptr & entry)
{
  if (!entry) {
    return;
  }
  {
    auto lck = mrpt::lockHelper(mtx_);
    if (closing_ || failed_) {
      return;
    }
    queue_.push_back(entry);
    constexpr std::size_t QUEUE_WARNING = 5000;
    if (queue_.size() > QUEUE_WARNING && !warned_queue_) {
      warned_queue_ = true;
      std::cerr << "[MapperInputRecorder] Warning: the disk does not keep up, " << queue_.size()
                << " entries waiting.\n";
    }
  }
  cv_.notify_one();
}

void MapperInputRecorder::record(const mrpt::obs::CObservation::ConstPtr & o)
{
  record(std::static_pointer_cast<mrpt::serialization::CSerializable>(
    std::const_pointer_cast<mrpt::obs::CObservation>(o)));
}

void MapperInputRecorder::close()
{
  {
    auto lck = mrpt::lockHelper(mtx_);
    closing_ = true;
  }
  cv_.notify_one();
  if (writer_.joinable()) {
    writer_.join();
  }
  if (out_) {
    out_->close();
  }
}

std::size_t MapperInputRecorder::written() const
{
  auto lck = mrpt::lockHelper(mtx_);
  return written_;
}

bool MapperInputRecorder::failed() const
{
  auto lck = mrpt::lockHelper(mtx_);
  return failed_;
}

void MapperInputRecorder::writer_loop()
{
  auto arch = mrpt::serialization::archiveFrom(*out_);
  for (;;) {
    mrpt::serialization::CSerializable::Ptr obj;
    {
      std::unique_lock<std::mutex> lck(mtx_);
      cv_.wait(lck, [this]() { return closing_ || !queue_.empty(); });
      if (queue_.empty()) {
        return;  // closing, and everything written
      }
      obj = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      arch << *obj;
    } catch (const std::exception & e) {
      // E.g. a full disk: the run goes on, without its recording.
      std::cerr << "[MapperInputRecorder] Recording stopped, cannot write: " << e.what() << "\n";
      auto lck = mrpt::lockHelper(mtx_);
      failed_ = true;
      queue_.clear();
      return;
    }
    auto lck = mrpt::lockHelper(mtx_);
    written_++;
  }
}

MapperInputReader::MapperInputReader(const std::string & file)
: in_(std::make_unique<mrpt::io::CFileGZInputStream>())
{
  ASSERTMSG_(in_->open(file), "Cannot read the mapper input log: " + file);
}

MapperInputReader::~MapperInputReader() = default;

std::optional<MapperInputEntry> MapperInputReader::next()
{
  if (truncated_) {
    return std::nullopt;
  }
  auto arch = mrpt::serialization::archiveFrom(*in_);
  mrpt::serialization::CSerializable::Ptr obj;
  try {
    obj = arch.ReadObject();
  } catch (const mrpt::serialization::CExceptionEOF &) {
    return std::nullopt;
  } catch (const std::exception & e) {
    // A recording cut short ends with a partial entry:
    ASSERTMSG_(read_ > 0, std::string("Cannot read the mapper input log: ") + e.what());
    std::cerr << "[MapperInputReader] Warning: the log ends with an incomplete entry, after "
              << read_ << " entries (a recording cut short?).\n";
    truncated_ = true;
    return std::nullopt;
  }
  read_++;
  if (auto sf = std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(obj); sf) {
    return MapperInputEntry(decode(*sf));
  }
  if (auto o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); o) {
    return MapperInputEntry(o);
  }
  THROW_EXCEPTION_FMT(
    "Unexpected object of class '%s' in the mapper input log.",
    obj ? obj->GetRuntimeClass()->className : "(null)");
}

}  // namespace mola::mapper
