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
 * @file   MapperInputLog.h
 * @brief  Record and read back everything a Mapper receives, to replay a run
 *         exactly (mola-mapper-cli).
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>

namespace mrpt::io
{
class CFileGZInputStream;
class CFileGZOutputStream;
}  // namespace mrpt::io

namespace mola::mapper
{
/** The gyroscope bias used for a keyframe (see Mapper::insert_keyframe()). */
struct RecordedGyroBias
{
  mrpt::math::TVector3D bias{0, 0, 0};  ///< [rad/s]
  double sigma = 0;                     ///< [rad/s]
  /// From a bias source; otherwise, zero bias with the configured sigma.
  bool from_source = false;
};

/** One keyframe insertion request, with the gyroscope bias it got. */
struct RecordedKeyframe
{
  SharedKeyframeMap::KeyframeInsertRequest request;
  RecordedGyroBias gyro_bias;
};

/** An entry of the log, in arrival order: a raw observation (GNSS, IMU), or a
 *  keyframe. */
using MapperInputEntry = std::variant<mrpt::obs::CObservation::Ptr, RecordedKeyframe>;

/** Writes the mapper input to a file (a gzip-compressed MRPT archive), from a
 *  background thread, so callers do not wait for the disk. Thread-safe. A write
 *  error stops the recording (failed()), not the process.
 *  Keyframes are stored as CSensoryFrame objects: their observations (point
 *  clouds deep-copied and embedded, so the log is self-contained) plus a
 *  CObservationComment ("mapper_keyframe_request") with the request fields, all
 *  numbers in exact (hexadecimal) form. */
class MapperInputRecorder
{
public:
  explicit MapperInputRecorder(const std::string & file);
  ~MapperInputRecorder();

  MapperInputRecorder(const MapperInputRecorder &) = delete;
  MapperInputRecorder & operator=(const MapperInputRecorder &) = delete;
  MapperInputRecorder(MapperInputRecorder &&) = delete;
  MapperInputRecorder & operator=(MapperInputRecorder &&) = delete;

  /** A keyframe as written to the log: built by the caller (it copies point
   *  clouds), then queued with record(). */
  [[nodiscard]] static mrpt::serialization::CSerializable::Ptr encode(const RecordedKeyframe & kf);

  /// Queues an observation, or an encoded keyframe, after the previous ones.
  void record(const mrpt::serialization::CSerializable::Ptr & entry);
  void record(const mrpt::obs::CObservation::ConstPtr & o);

  /// Writes what is queued and closes the file. Called by the destructor.
  void close();

  [[nodiscard]] std::size_t written() const;
  [[nodiscard]] bool failed() const;

private:
  std::unique_ptr<mrpt::io::CFileGZOutputStream> out_;
  std::thread writer_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<mrpt::serialization::CSerializable::Ptr> queue_;
  bool closing_ = false;
  bool failed_ = false;
  bool warned_queue_ = false;
  std::size_t written_ = 0;

  void writer_loop();
};

/** Reads a file written by MapperInputRecorder, entry by entry. */
class MapperInputReader
{
public:
  explicit MapperInputReader(const std::string & file);
  ~MapperInputReader();

  MapperInputReader(const MapperInputReader &) = delete;
  MapperInputReader & operator=(const MapperInputReader &) = delete;
  MapperInputReader(MapperInputReader &&) = delete;
  MapperInputReader & operator=(MapperInputReader &&) = delete;

  /** The next entry, or nothing at the end of the file. A log cut short (a
   *  recording process killed) ends at its last complete entry: see truncated(). */
  [[nodiscard]] std::optional<MapperInputEntry> next();

  [[nodiscard]] bool truncated() const { return truncated_; }

private:
  std::unique_ptr<mrpt::io::CFileGZInputStream> in_;
  std::size_t read_ = 0;
  bool truncated_ = false;
};

}  // namespace mola::mapper
