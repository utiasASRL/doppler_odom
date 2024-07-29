#pragma once

#include "doppler_odom/dataset.hpp"
#include "doppler_odom/calib/doppler_image_calib.hpp"

namespace doppler_odom {

class AevaHQDataset : public Dataset {
 public:
  struct Options : public Dataset::Options{
    // AevaHQDataset specific options
    DopplerImageCalib::Options dcalib_options;
    std::vector<std::vector<double>> xi_sv;
    std::vector<std::vector<double>> gyro_ivar;
    int nframes_gbias_calib;
    std::vector<bool> active_lidars;
    std::vector<bool> active_gyros;

    // set parameters from yaml
    void setParamsFromYaml(const YAML::Node& config) override {
      // set base parameters
      this->setBaseParamsFromYaml(config);

      // set child parameters
      dcalib_options.setParamsFromYaml(config);
      xi_sv = config["dataset_options"]["xi_sv"].as<std::vector<std::vector<double>>>();
      gyro_ivar = config["dataset_options"]["gyro_ivar"].as<std::vector<std::vector<double>>>();
      nframes_gbias_calib = config["dataset_options"]["nframes_gbias_calib"].as<int>();
      this->active_lidars = config["dataset_options"]["active_lidars"].as<std::vector<bool>>();
      this->active_gyros = config["dataset_options"]["active_gyros"].as<std::vector<bool>>();
    }
  };

  AevaHQDataset(const Options& options) : options_(options) {
    if (options_.all_sequences)
      sequences_ = SEQUENCES;
    else
      sequences_.emplace_back(options_.sequence);
  }

  bool hasNext() const override { return next_sequence_ < sequences_.size(); }
  Sequence::Ptr next() override;

 private:
  Options options_;
  std::vector<std::string> sequences_;
  size_t next_sequence_ = 0;

 private:
  static inline std::vector<std::string> SEQUENCES{
      "routeA_run4_binary",
      "routeA_run5_binary",
      "routeA_run6_binary",
      "routeB_run1_binary",
      "routeB_run2_binary",
      "routeB_run3_binary",
      "routeC_run1_binary",
      "routeC_run2_binary",
      "routeC_run3_binary"
  };

  DOPPLER_ODOM_REGISTER_DATASET("aevahq", AevaHQDataset);
};

class AevaHQSequence : public Sequence {
 public:
  AevaHQSequence(const AevaHQDataset::Options& options);
  ~AevaHQSequence() = default;

  std::string name() const override { return options_.sequence; }
  int currFrame() const override { return curr_frame_[0] - init_frame_[0]; }
  int numFrames() const override { return last_frame_[0] - init_frame_[0]; }
  bool hasNext() const override { return curr_frame_[0] < last_frame_[0]; }
  Pointcloud next(double& start_time, double& end_time) override;
  std::vector<Eigen::MatrixXd> nextGyro(const double& start_time, const double& end_time) override;
  Pointcloud preprocessFrame(Pointcloud& frame, double start_time, double end_time) override;

  bool hasGroundTruth() const override { return false; }  // TODO

  void save(const std::string &path, const Trajectory &trajectory, const std::vector<Eigen::Matrix4d> &poses) const override;

 private:
  AevaHQDataset::Options options_;
  std::string dir_path_[4];
  std::vector<std::vector<std::string>> filenames_;
  int64_t initial_timestamp_micro_;
  int init_frame_[4] = {0};
  int curr_frame_[4] = {0};
  int last_frame_[4] = {std::numeric_limits<int>::max()};  // exclusive bound

  // calib
  DopplerImageCalib::ConstPtr doppler_image_calib_;

  // gyro measurements
  std::vector<Eigen::MatrixXd> gyro_data_;
  std::vector<Eigen::Vector3d> const_gyro_bias_;
};

}  // namespace doppler_odom
