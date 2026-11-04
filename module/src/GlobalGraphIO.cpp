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
 * @file   GlobalGraphIO.cpp
 * @brief  Save and load the input of the global graph, to re-optimize it offline.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_mapper/GlobalGraphIO.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3DQuat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace mola::mapper
{
namespace
{
void write_pose(std::ostream & f, const mrpt::poses::CPose3D & p)
{
  f << " " << p.x() << " " << p.y() << " " << p.z() << " " << p.yaw() << " " << p.pitch() << " "
    << p.roll();
}

void write_cov(std::ostream & f, const mrpt::math::CMatrixDouble66 & c)
{
  for (int r = 0; r < 6; r++) {
    for (int k = 0; k < 6; k++) {
      f << " " << c(r, k);
    }
  }
}

mrpt::poses::CPose3D read_pose(std::istream & s)
{
  double x = 0;
  double y = 0;
  double z = 0;
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
  s >> x >> y >> z >> yaw >> pitch >> roll;
  return mrpt::poses::CPose3D(x, y, z, yaw, pitch, roll);
}

mrpt::math::CMatrixDouble66 read_cov(std::istream & s)
{
  mrpt::math::CMatrixDouble66 c;
  for (int r = 0; r < 6; r++) {
    for (int k = 0; k < 6; k++) {
      s >> c(r, k);
    }
  }
  return c;
}

}  // namespace

void save_global_graph_input(const GlobalGraphInput & in, const std::string & prefix)
{
  {
    std::ofstream f(prefix + ".keyframes.txt");
    ASSERTMSG_(f.is_open(), "Cannot write " + prefix + ".keyframes.txt");
    f << std::setprecision(15);
    f << "# idx stamp source pose_in_source(x y z yaw pitch roll) cov(36) initial_guess(6)"
         " [gyro_rotation_from_prev(yaw pitch roll) sigma]\n";
    mrpt::maps::CSimpleMap aux;
    for (std::size_t i = 0; i < in.keyframes.size(); i++) {
      const auto & kf = in.keyframes[i];
      f << i << " " << std::fixed << std::setprecision(6) << mrpt::Clock::toDouble(kf.stamp)
        << std::defaultfloat << std::setprecision(15) << " " << kf.source_frame;
      write_pose(f, kf.pose_in_source.mean);
      write_cov(f, kf.pose_in_source.cov);
      write_pose(f, kf.initial_guess);
      if (kf.gyro_rotation_from_prev) {
        const auto & g = *kf.gyro_rotation_from_prev;
        f << " " << g.yaw() << " " << g.pitch() << " " << g.roll() << " " << kf.gyro_rotation_sigma;
      }
      f << "\n";
      auto sf = kf.aux_observations
                  ? std::make_shared<mrpt::obs::CSensoryFrame>(*kf.aux_observations)
                  : mrpt::obs::CSensoryFrame::Create();
      aux.insert(mrpt::poses::CPose3DPDFGaussian::Create(kf.initial_guess), sf);
    }
    ASSERT_(aux.saveToFile(prefix + ".aux.simplemap"));
  }
  {
    std::ofstream f(prefix + ".loops.txt");
    ASSERTMSG_(f.is_open(), "Cannot write " + prefix + ".loops.txt");
    f << std::setprecision(15);
    f << "# from to relative_pose(x y z yaw pitch roll) cov(36)\n";
    for (const auto & lc : in.loops) {
      f << lc.from << " " << lc.to;
      write_pose(f, lc.relative_pose.mean);
      write_cov(f, lc.relative_pose.cov);
      f << "\n";
    }
  }
  if (in.geodetic_datum) {
    std::ofstream f(prefix + ".datum.txt");
    ASSERTMSG_(f.is_open(), "Cannot write " + prefix + ".datum.txt");
    f << std::setprecision(15) << in.geodetic_datum->lat.decimal_value << " "
      << in.geodetic_datum->lon.decimal_value << " " << in.geodetic_datum->height << "\n";
  } else {
    std::remove((prefix + ".datum.txt").c_str());
  }
  if (in.T_enu_to_map_guess) {
    std::ofstream f(prefix + ".enu_to_map_guess.txt");
    ASSERTMSG_(f.is_open(), "Cannot write " + prefix + ".enu_to_map_guess.txt");
    f << std::setprecision(15);
    write_pose(f, *in.T_enu_to_map_guess);
    f << "\n";
  } else {
    // Not one of an earlier input saved with the same prefix:
    std::remove((prefix + ".enu_to_map_guess.txt").c_str());
  }
}

GlobalGraphInput load_global_graph_input(const std::string & prefix)
{
  GlobalGraphInput in;

  mrpt::maps::CSimpleMap aux;
  ASSERTMSG_(
    aux.loadFromFile(prefix + ".aux.simplemap"), "Cannot read " + prefix + ".aux.simplemap");

  std::ifstream f(prefix + ".keyframes.txt");
  ASSERTMSG_(f.is_open(), "Cannot read " + prefix + ".keyframes.txt");
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream s(line);
    std::size_t idx = 0;
    double t = 0;
    GlobalGraphKeyframe kf;
    s >> idx >> t >> kf.source_frame;
    kf.stamp = mrpt::Clock::fromDouble(t);
    kf.pose_in_source.mean = read_pose(s);
    kf.pose_in_source.cov = read_cov(s);
    kf.initial_guess = read_pose(s);
    // Optional (newer files): gyroscope rotation from the previous keyframe.
    double gYaw = 0;
    double gPitch = 0;
    double gRoll = 0;
    double gSigma = 0;
    if (s >> gYaw >> gPitch >> gRoll >> gSigma) {
      kf.gyro_rotation_from_prev = mrpt::poses::CPose3D(0, 0, 0, gYaw, gPitch, gRoll);
      kf.gyro_rotation_sigma = gSigma;
    }
    ASSERT_EQUAL_(idx, in.keyframes.size());
    ASSERT_LT_(idx, aux.size());
    kf.aux_observations = aux.get(idx).sf;
    in.keyframes.push_back(kf);
  }

  std::ifstream fl(prefix + ".loops.txt");
  while (fl.is_open() && std::getline(fl, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream s(line);
    GlobalGraphLoopEdge lc;
    s >> lc.from >> lc.to;
    lc.relative_pose.mean = read_pose(s);
    lc.relative_pose.cov = read_cov(s);
    in.loops.push_back(lc);
  }

  std::ifstream fd(prefix + ".datum.txt");
  if (fd.is_open()) {
    double lat = 0;
    double lon = 0;
    double h = 0;
    fd >> lat >> lon >> h;
    in.geodetic_datum = mrpt::topography::TGeodeticCoords(lat, lon, h);
  }

  std::ifstream fg(prefix + ".enu_to_map_guess.txt");
  if (fg.is_open()) {
    in.T_enu_to_map_guess = read_pose(fg);
  }
  return in;
}

void save_georeference(const mola::Georeferencing & g, const std::string & file)
{
  std::ofstream f(file);
  ASSERTMSG_(f.is_open(), "Cannot write " + file);
  f << std::setprecision(12);
  const auto & m = g.T_enu_to_map.mean;
  f << "latitude: " << g.geo_coord.lat.decimal_value << "\n"
    << "longitude: " << g.geo_coord.lon.decimal_value << "\n"
    << "height: " << g.geo_coord.height << "\n"
    << "T_enu_to_map: [" << m.x() << ", " << m.y() << ", " << m.z() << ", " << m.yaw() << ", "
    << m.pitch() << ", " << m.roll() << "]  # x y z yaw pitch roll [m, rad]\n"
    << "T_enu_to_map_sigmas: [";
  for (int i = 0; i < 6; i++) {
    f << std::sqrt(std::max(0.0, g.T_enu_to_map.cov(i, i))) << (i < 5 ? ", " : "]\n");
  }
}

void save_tum_trajectory(
  const std::vector<std::pair<mrpt::Clock::time_point, mrpt::poses::CPose3D>> & poses,
  const std::string & file)
{
  std::ofstream f(file);
  ASSERTMSG_(f.is_open(), "Cannot write " + file);
  f << std::fixed << std::setprecision(6);
  for (const auto & [t, p] : poses) {
    const mrpt::poses::CPose3DQuat q(p);
    f << mrpt::Clock::toDouble(t) << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << q.quat().x() << " " << q.quat().y() << " " << q.quat().z() << " " << q.quat().r() << "\n";
  }
}

}  // namespace mola::mapper
