#pragma once

#include <queue>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "doppler_odom/point.hpp"

namespace doppler_odom {

// Voxel
// Note: Coordinates range is in [-32 768, 32 767]
struct Voxel {
  Voxel() = default;

  Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

  bool operator==(const Voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

  inline bool operator<(const Voxel &vox) const {
    return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
  }

  inline static Voxel Coordinates(const Eigen::Vector3d &point, double voxel_size) {
    return {short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size)};
  }

  short x;
  short y;
  short z;
};

struct Voxel2D {
  Voxel2D() = default;

  Voxel2D(short x, short y) : x(x), y(y) {}

  bool operator==(const Voxel2D &vox) const { return x == vox.x && y == vox.y; }

  inline bool operator<(const Voxel2D &vox) const {
    return x < vox.x || (x == vox.x && y < vox.y);
  }

  // inline static Voxel Coordinates(const Eigen::Vector3d &point, double voxel_size) {
  //   return {short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size)};
  // }

  short x;
  short y;
};

}  // namespace doppler_odom