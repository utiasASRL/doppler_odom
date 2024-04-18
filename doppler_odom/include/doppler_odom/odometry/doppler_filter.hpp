#pragma once

#include <fstream>

#include "doppler_odom/odometry.hpp"

namespace doppler_odom {

class DopplerFilter : public Odometry {
 public:

  struct Options : public Odometry::Options {
    std::vector<Eigen::Vector3d> const_gyro_bias; // constant gyro bias (1 for each sensor)
  };

  DopplerFilter(const Options &options);
  ~DopplerFilter();

  void solveFrame(const Pointcloud& frame, const std::vector<Eigen::MatrixXd>& gyro) override;
  Pointcloud preprocessFrame(Pointcloud &frame, const double& start_time, const double& end_time) override;
  Pointcloud ransacFrame(const Pointcloud &frame) override;
  Eigen::Matrix4d integrateForPose() override;
  
  std::vector<double> getLatestFrameTimes() override {
    std::vector<double> out;
    out.push_back(trajectory_.back().begin_timestamp);
    out.push_back(trajectory_.back().end_timestamp);
    return out;
  }

 private:
  void initializeTimestamp(int index_frame, const std::vector<Pointcloud> &const_frame);
  const Options options_;

  // precompute
  Eigen::Matrix<double, 12, 12> wnoa_lhs_;

  DOPPLER_ODOM_REGISTER_ODOMETRY("doppler_filter", DopplerFilter);
};

}  // namespace doppler_odom