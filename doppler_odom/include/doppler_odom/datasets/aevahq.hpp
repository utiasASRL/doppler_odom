#pragma once

#include "doppler_odom/dataset.hpp"

namespace doppler_odom {

class AevaHQSequence : public Sequence {
 public:
  AevaHQSequence(const Options& options);

  int currFrame() const override { return curr_frame_[0] - init_frame_[0]; }
  int numFrames() const override { return last_frame_[0] - init_frame_[0]; }
  bool hasNext() const override { return curr_frame_[0] < last_frame_[0]; }
  Pointcloud next(double& start_time, double& end_time) override;
  std::vector<Eigen::MatrixXd> next_gyro(const double& start_time, const double& end_time) override;

  bool hasGroundTruth() const override { return false; }  // TODO

  void save(const std::string &path, const Trajectory &trajectory, const std::vector<Eigen::Matrix4d> &poses) const override;

 private:
  std::string dir_path_[4];
  std::vector<std::vector<std::string>> filenames_;
  int64_t initial_timestamp_micro_;
  // int init_frame_ = 0;
  // int curr_frame_ = 0;
  int init_frame_[4] = {0};
  int curr_frame_[4] = {0};
  int last_frame_[4] = {std::numeric_limits<int>::max()};  // exclusive bound

  // gyro measurements
  std::vector<Eigen::MatrixXd> gyro_data_;
};

class AevaHQDataset : public Dataset {
 public:
  AevaHQDataset(const Options& options) : Dataset(options) {
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
    return std::make_shared<AevaHQSequence>(options);
  }

 private:
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

}  // namespace doppler_odom
