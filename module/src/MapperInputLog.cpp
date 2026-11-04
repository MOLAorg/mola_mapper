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
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace mola::mapper
{
namespace
{
const char * KEYFRAME_LABEL = "mapper_keyframe_request";

/** Doubles in hexadecimal form: they read back exactly. */
void write_numbers(std::ostream & s, const std::vector<double> & v)
{
  for (const double x : v) {
    s << ' ' << std::hexfloat << x;
  }
  s << std::defaultfloat << '\n';
}

std::vector<double> read_numbers(const std::string & rest)
{
  std::vector<double> v;
  std::istringstream s(rest);
  std::string tok;
  while (s >> tok) {
    v.push_back(std::strtod(tok.c_str(), nullptr));  // strtod parses hexfloats
  }
  return v;
}

mrpt::obs::CSensoryFrame::Ptr encode(const RecordedKeyframe & kf)
{
  const auto & r = kf.request;
  std::ostringstream s;
  s << "source_frame_id " << r.source_frame_id << '\n';
  s << "quality";
  write_numbers(s, {r.quality});
  // The 3x4 matrix, not angles, so the rotation reads back exactly:
  const auto h = r.pose_in_source.mean.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>();
  std::vector<double> pose;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      pose.push_back(h(i, j));
    }
  }
  s << "pose";
  write_numbers(s, pose);
  std::vector<double> cov;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov.push_back(r.pose_in_source.cov(i, j));
    }
  }
  s << "cov";
  write_numbers(s, cov);
  const auto & b = kf.gyro_bias;
  s << "gyro_bias";
  write_numbers(s, {b.bias.x, b.bias.y, b.bias.z, b.sigma});

  auto c = mrpt::obs::CObservationComment::Create();
  c->sensorLabel = KEYFRAME_LABEL;
  c->timestamp = r.timestamp;
  c->text = s.str();

  auto sf = mrpt::obs::CSensoryFrame::Create(r.observations);
  sf->insert(c);
  return sf;
}

RecordedKeyframe decode(const mrpt::obs::CSensoryFrame & sf)
{
  RecordedKeyframe kf;
  auto & r = kf.request;
  bool found = false;
  for (std::size_t i = 0; i < sf.size(); i++) {
    const auto o = sf.getObservationByIndex(i);
    const auto c = std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o);
    if (!c || c->sensorLabel != KEYFRAME_LABEL) {
      r.observations.insert(std::const_pointer_cast<mrpt::obs::CObservation>(o));
      continue;
    }
    found = true;
    r.timestamp = c->timestamp;
    std::istringstream lines(c->text);
    std::string line;
    while (std::getline(lines, line)) {
      const auto sp = line.find(' ');
      const std::string key = line.substr(0, sp);
      const std::string rest = sp == std::string::npos ? "" : line.substr(sp + 1);
      if (key == "source_frame_id") {
        r.source_frame_id = rest;
        continue;
      }
      const auto v = read_numbers(rest);
      if (key == "quality" && v.size() == 1) {
        r.quality = v[0];
      } else if (key == "pose" && v.size() == 12) {
        mrpt::math::CMatrixDouble44 h = mrpt::math::CMatrixDouble44::Identity();
        for (int k = 0; k < 12; k++) {
          h(k / 4, k % 4) = v[k];
        }
        r.pose_in_source.mean = mrpt::poses::CPose3D(h);
      } else if (key == "cov" && v.size() == 36) {
        for (int k = 0; k < 36; k++) {
          r.pose_in_source.cov(k / 6, k % 6) = v[k];
        }
      } else if (key == "gyro_bias" && v.size() == 4) {
        kf.gyro_bias.bias = {v[0], v[1], v[2]};
        kf.gyro_bias.sigma = v[3];
      }
    }
  }
  ASSERTMSG_(found, "Sensory frame without a keyframe request in the mapper input log.");
  return kf;
}

}  // namespace

MapperInputRecorder::MapperInputRecorder(const std::string & file)
: out_(std::make_unique<mrpt::io::CFileGZOutputStream>())
{
  // Fast compression: the scans dominate, and writing must keep up with a live run.
  ASSERTMSG_(out_->open(file, 1), "Cannot write the mapper input log: " + file);
  writer_ = std::thread([this]() { writer_loop(); });
}

MapperInputRecorder::~MapperInputRecorder() { close(); }

void MapperInputRecorder::record(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!o) {
    return;
  }
  {
    auto lck = mrpt::lockHelper(mtx_);
    if (closing_) {
      return;
    }
    queue_.push_back(std::const_pointer_cast<mrpt::obs::CObservation>(o));
  }
  cv_.notify_one();
}

void MapperInputRecorder::record(const RecordedKeyframe & kf)
{
  auto sf = encode(kf);
  {
    auto lck = mrpt::lockHelper(mtx_);
    if (closing_) {
      return;
    }
    queue_.push_back(sf);
  }
  cv_.notify_one();
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
    arch << *obj;
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
  auto arch = mrpt::serialization::archiveFrom(*in_);
  mrpt::serialization::CSerializable::Ptr obj;
  try {
    obj = arch.ReadObject();
  } catch (const mrpt::serialization::CExceptionEOF &) {
    return std::nullopt;
  }
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
