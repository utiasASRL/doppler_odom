#pragma once

#include "doppler_odom/dataset.hpp"

namespace doppler_odom {

class BoreasAevaSequence : public Sequence {
 public:
  BoreasAevaSequence(const Options& options);

  int currFrame() const override { return curr_frame_; }
  int numFrames() const override { return last_frame_ - init_frame_; }
  bool hasNext() const override { return curr_frame_ < last_frame_; }
  std::vector<Pointcloud> next(double& start_time, double& end_time) override;
  std::vector<Eigen::MatrixXd> next_gyro(const double& start_time, const double& end_time) override;

  bool hasGroundTruth() const override { return false; }  // TODO

  void save(const std::string &path, const Trajectory &trajectory, const std::vector<Eigen::Matrix4d> &poses) const override;

 private:
  std::string dir_path_;
  std::vector<std::string> filenames_;
  int64_t initial_timestamp_micro_;
  int init_frame_ = 0;
  int curr_frame_ = 0;
  int last_frame_ = std::numeric_limits<int>::max();  // exclusive bound

  // gyro measurements
  std::vector<Eigen::MatrixXd> gyro_data_;
  std::vector<Eigen::Vector3d> const_gyro_bias_;  // const gyro bias
};

class BoreasAevaDataset : public Dataset {
 public:
  BoreasAevaDataset(const Options& options) : Dataset(options) {
    if (options_.all_sequences)
      sequences_ = SEQUENCES;
    else
      sequences_.emplace_back(options_.sequence);
  }

  bool hasNext() const override { return next_sequence_ < sequences_.size(); }
  Sequence::Ptr next() override {
    if (!hasNext()) return nullptr;
    Sequence::Options options(options_);
    options.sequence = sequences_[next_sequence_++];
    return std::make_shared<BoreasAevaSequence>(options);
  }

 private:
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

}  // namespace doppler_odom
