#pragma once

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include "doppler_odom/point.hpp"

namespace doppler_odom {

class DopplerImageCalib {
 public:
  using Ptr = std::shared_ptr<DopplerImageCalib>;
  using ConstPtr = std::shared_ptr<const DopplerImageCalib>;
  struct Options {
    std::string root_path;
    double azimuth_res = 0.2 * M_PI / 180.0;   // rad
    double azimuth_start = -0.872665;
    double azimuth_end = 0.872665;
    int num_rows = 80;
    int num_cols = 501;   
    // int num_sensors = 1;  // TODO: shouldn't be hardcoded
    std::vector<bool> active_sensors;

    void setParamsFromYaml(const YAML::Node& config) {
      this->root_path = config["doppler_options"]["root_path"].as<std::string>();
      this->azimuth_res = config["doppler_options"]["azimuth_res"].as<double>();
      this->azimuth_start = config["doppler_options"]["azimuth_start"].as<double>();
      this->azimuth_end = config["doppler_options"]["azimuth_end"].as<double>();
      this->num_rows = config["doppler_options"]["num_rows"].as<int>();
      this->num_cols = config["doppler_options"]["num_cols"].as<int>();
    }
  };

  DopplerImageCalib(const Options& options);
  // ~DopplerImageCalib();

  std::vector<Point3D> calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const;

 protected:
  const Options options_;
  std::vector<Eigen::MatrixXd> elevation_order_;
  std::vector<std::vector<Eigen::MatrixXd>> elevation_order_by_beam_id_;

  // TODO: load different calibration models
  std::vector<std::vector<Eigen::MatrixXd>> weights_;
};

} // namespace doppler_odom