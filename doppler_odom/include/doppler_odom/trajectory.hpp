#pragma once

#include "doppler_odom/point.hpp"

namespace doppler_odom {

using ArrayMatrix4d = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
using ArrayPoses = ArrayMatrix4d;

struct TrajectoryFrame {
  TrajectoryFrame() = default;

  double begin_timestamp = 0.0;
  double end_timestamp = 1.0;

  Eigen::Matrix<double, 6, 1> varpi;  // estimate at end_timestamp

//   std::vector<Point3D> points;
};

using Trajectory = std::vector<TrajectoryFrame>;

}  // namespace steam_icp