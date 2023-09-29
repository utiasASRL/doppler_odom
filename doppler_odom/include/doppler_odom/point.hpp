#pragma once

#include <Eigen/Dense>

namespace doppler_odom {

// A Point3D
struct Point3D {
  Eigen::Vector3d pt;      // point data
  double radial_velocity = 0.0;  // Radial velocity of the point
  // double alpha_timestamp = 0.0;  // Relative timestamp in the frame in [0.0, 1.0]
  double timestamp = 0.0;        // The absolute timestamp (if applicable)
  // double azimuth;             // should be available from raw data, but we don't record it                
  // double elevation;           // should be available from raw data, but we don't record it
  double range;               // should be available from raw data, but we don't record it
  int beam_id = -1;              // The beam id of the point
};

using Pointcloud = std::vector<Point3D>;

}  // namespace doppler_odom
