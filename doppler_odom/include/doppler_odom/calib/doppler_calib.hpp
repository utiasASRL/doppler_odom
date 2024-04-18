#pragma once

#include <memory>
#include <string>
#include <vector>
#include <random>

#include "doppler_odom/point.hpp"

namespace doppler_odom {

class DopplerCalib {
 public:
  using Ptr = std::shared_ptr<DopplerCalib>;
  using ConstPtr = std::shared_ptr<const DopplerCalib>;
  struct Options {
    std::string root_path;
    double azimuth_res = 0.2 * M_PI / 180.0;   // rad
    double azimuth_start = -0.872665;
    double azimuth_end = 0.872665;
    // int num_rows = 64;
    // int num_cols = 551;   
    int num_rows = 80;
    int num_cols = 501;   
    // int num_sensors = 1;  // TODO: shouldn't be hardcoded
    std::vector<bool> active_sensors;
  };

  DopplerCalib(const Options& options);
  // ~DopplerCalib();

  std::vector<Point3D> calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const;

 protected:
  const Options options_;
  std::vector<Eigen::MatrixXd> elevation_order_;
  std::vector<std::vector<Eigen::MatrixXd>> elevation_order_by_beam_id_;

  // TODO: load different calibration models
  std::vector<std::vector<Eigen::MatrixXd>> weights_;
};

} // namespace doppler_odom