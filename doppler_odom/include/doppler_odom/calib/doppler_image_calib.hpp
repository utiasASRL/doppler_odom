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
    std::string model_name;
    double azimuth_res = 0.2 * M_PI / 180.0;   // rad
    double azimuth_start = -0.872665;
    double azimuth_end = 0.872665;
    int num_rows = 80;
    int num_cols = 501;   
    int downsample_steps = 1;
    // int num_sensors = 1;  // TODO: shouldn't be hardcoded
    std::vector<bool> active_sensors;

    void setParamsFromYaml(const YAML::Node& config) {
      this->root_path = config["doppler_options"]["root_path"].as<std::string>();
      this->model_name = config["doppler_options"]["model"].as<std::string>();
      this->azimuth_res = config["doppler_options"]["azimuth_res"].as<double>();
      this->azimuth_start = config["doppler_options"]["azimuth_start"].as<double>();
      this->azimuth_end = config["doppler_options"]["azimuth_end"].as<double>();
      this->num_rows = config["doppler_options"]["num_rows"].as<int>(); // TODO: should we be reading this from model parameters?
      this->num_cols = config["doppler_options"]["num_cols"].as<int>();
      this->downsample_steps = config["dataset_options"]["downsample_steps"].as<int>();
    }
  };

  DopplerImageCalib(const Options& options);
  // ~DopplerImageCalib();

  std::vector<Point3D> calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const;

 protected:
  Options options_;

  // calibration model weights
  using ImgWeight = std::vector<std::vector<Eigen::VectorXd>>; // (elevation) x (azimuth) x (weight dim)
  std::vector<std::vector<ImgWeight>> bias_weights_; // (# sensors) x (# face ids)
  std::vector<std::vector<ImgWeight>> var_weights_; // (# sensors) x (# face ids)
  std::vector<std::string> bias_features_;
  std::vector<std::string> var_features_;
  int bias_porder_;
  int var_porder_;
  bool calc_dop_median_ = false;
  bool calc_pseudo_var_ = false;
  int pseudo_var_hwidth_ = 5;

  void initImgWeight(bool set_dims, const std::string& dim_txt, const std::string& binary, std::vector<std::vector<ImgWeight>>& weights);
  void buildFeatVec(Eigen::VectorXd& feat, const Point3D& point, const std::vector<std::string>& feat_string, 
      double dop_median_, double dop_pseudovar) const;
  double computeModel(const Eigen::VectorXd& feat, const Eigen::VectorXd& weights, int polyorder) const;
  bool computePseudovar(double& pseudovar, const std::vector<const Point3D*>& img_row, int c, int hwidth, double tol) const;
};

} // namespace doppler_odom