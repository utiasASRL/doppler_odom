#pragma once

#include <Eigen/Dense>

namespace doppler_odom {

// A Point3D
struct Point3D {
  Eigen::Vector3d pt;            // point data
  double radial_velocity = 0.0;  // radial velocity of the point
  double timestamp = 0.0;        // the absolute timestamp (if applicable)
  double range;                  // should be available from raw data, but we don't record it
  double intensity;
  int beam_id = -1;              // beam id of the point
  int face_id = -1;              // face id of polygon mirror (available on aeries II)
  int sensor_id = 0;             // distinguishes between multiple sensors (first sensor should be set to 0)
  int line_id = -1;              // the horizontal scan line the measurement belongs to, starting from the top. Used to project into image.
};

using Pointcloud = std::vector<Point3D>;

}  // namespace doppler_odom
