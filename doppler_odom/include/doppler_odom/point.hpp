#pragma once

#include <Eigen/Dense>

namespace doppler_odom {

// A Point3D
struct Point3D {
  Eigen::Vector3d pt;            // point data
  double radial_velocity = 0.0;  // Radial velocity of the point
  double timestamp = 0.0;        // The absolute timestamp (if applicable)
  double range;                  // should be available from raw data, but we don't record it
  int beam_id = -1;              // The beam id of the point
  int sensor_id = 0;            // Distinguishes between multiple sensors (first sensor should be set to 0)
};

using Pointcloud = std::vector<Point3D>;

}  // namespace doppler_odom
