#pragma once

#include "doppler_odom/dataset.hpp"
#include "doppler_odom/calib/doppler_image_calib.hpp"

namespace doppler_odom {

class BoreasAevaDataset : public Dataset {
 public:
  struct Options : public Dataset::Options{
    // BoreasAevaDataset-specific options
    DopplerImageCalib::Options dcalib_options;
    
    // set parameters from yaml
    void setParamsFromYaml(const YAML::Node& config) override {
      // set base parameters
      this->setBaseParamsFromYaml(config);

      // set child parameters
      dcalib_options.setParamsFromYaml(config);
    }
  };

  BoreasAevaDataset(const Options& options) : options_(options) {
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
      "2023_02_15_1",
      "2023_02_15_2",
      "2023_02_15_3",
      "2023_02_15_4",
      "2023_02_15_5"
  };

  DOPPLER_ODOM_REGISTER_DATASET("boreas_aeva", BoreasAevaDataset);
};

class BoreasAevaSequence : public Sequence {
 public:
  BoreasAevaSequence(const BoreasAevaDataset::Options& options);
  ~BoreasAevaSequence() = default;

  std::string name() const override { return options_.sequence; }
  int currFrame() const override { return curr_frame_; }
  int numFrames() const override { return last_frame_ - init_frame_; }
  bool hasNext() const override { return curr_frame_ < last_frame_; }
  Pointcloud next(double& start_time, double& end_time) override;
  std::vector<Eigen::MatrixXd> nextGyro(const double& start_time, const double& end_time) override;
  Pointcloud preprocessFrame(Pointcloud& frame, double start_time, double end_time) override;

  bool hasGroundTruth() const override { return false; }  // TODO
  void save(const std::string &path, const Trajectory &trajectory, const std::vector<Eigen::Matrix4d> &poses) const override;

 private:
  BoreasAevaDataset::Options options_;
  std::string dir_path_;
  std::vector<std::string> filenames_;
  int64_t initial_timestamp_micro_;
  int init_frame_ = 0;
  int curr_frame_ = 0;
  int last_frame_ = std::numeric_limits<int>::max();  // exclusive bound

  // calib
  DopplerImageCalib::ConstPtr doppler_image_calib_;

  // gyro measurements
  std::vector<Eigen::MatrixXd> gyro_data_;
};

}  // namespace doppler_odom
