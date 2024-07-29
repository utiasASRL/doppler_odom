#pragma once

#include "doppler_odom/dataset.hpp"

namespace doppler_odom {

class ExampleDataset : public Dataset {
 public:
  struct Options : public Dataset::Options{
    // ExampleDataset specific options
    /* add option parameters here */
    std::string example_string;
    std::vector<std::string> example_string_vec;
    double example_double;
    std::vector<double> example_double_vec;

    // set parameters from yaml
    void setParamsFromYaml(const YAML::Node& config) override {
      // set base parameters
      this->setBaseParamsFromYaml(config);

      // set child parameters
      /* add any options to load from yaml config file here */
      example_string = config["dataset_options"]["example_string"].as<std::string>();
      example_string_vec = config["dataset_options"]["example_string_list"].as<std::vector<std::string>>();
      example_double = config["dataset_options"]["example_double"].as<double>();
      example_double_vec = config["dataset_options"]["example_double_list"].as<std::vector<double>>();
    }
  };

  ExampleDataset(const Options& options) : options_(options) {
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
      /* list all your sequences here */ 
      "sim_example_sequence1",
      "sim_example_sequence2"
  };

  DOPPLER_ODOM_REGISTER_DATASET("example", ExampleDataset);
};

class ExampleSequence : public Sequence {
 public:
  ExampleSequence(const ExampleDataset::Options& options);
  ~ExampleSequence() = default;

  std::string name() const override { return options_.sequence; }
  int currFrame() const override { return curr_frame_ - init_frame_; }
  int numFrames() const override { return last_frame_ - init_frame_; }
  bool hasNext() const override { return curr_frame_ < last_frame_; }
  Pointcloud next(double& start_time, double& end_time) override;
  std::vector<Eigen::MatrixXd> nextGyro(const double& start_time, const double& end_time) override;
  Pointcloud preprocessFrame(Pointcloud& frame, double start_time, double end_time) override;

  bool hasGroundTruth() const override { return false; }  // TODO

  void save(const std::string &path, const Trajectory &trajectory, const std::vector<Eigen::Matrix4d> &poses) const override;

 private:
  ExampleDataset::Options options_;
  int init_frame_ = 0;
  int curr_frame_ = 0;
  int last_frame_ = std::numeric_limits<int>::max();  // exclusive bound

  Eigen::Matrix<double, 6, 1> const_gt_vel_;  // constant 6DOF body-velocity for our simulation
};

}  // namespace doppler_odom
