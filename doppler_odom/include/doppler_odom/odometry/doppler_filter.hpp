#pragma once

#include <fstream>

#include "doppler_odom/odometry.hpp"

namespace doppler_odom {

class DopplerFilter : public Odometry {
 public:

  struct Options : public Odometry::Options {
    size_t downsample_steps = 1;
  };

  DopplerFilter(const Options &options);
  ~DopplerFilter();

  void solveFrame(const Pointcloud& frame, const std::vector<Eigen::MatrixXd>& gyro) override;
  Pointcloud preprocessFrame(Pointcloud &frame, double start_time, double end_time) override;
  Pointcloud ransacFrame(const Pointcloud &frame, const std::vector<Eigen::MatrixXd>& gyro) override;
  Eigen::Matrix4d integrateForPose() override;
  
  std::vector<double> getLatestFrameTimes() override {
    std::vector<double> out;
    out.push_back(trajectory_.back().begin_timestamp);
    out.push_back(trajectory_.back().end_timestamp);
    return out;
  }

 private:
  const Options options_;

  // precompute WNOA prior
  Eigen::Matrix<double, 12, 12> wnoa_lhs_;

  // precomputed measurement model (to avoid repeated calculations in RANSAC and main solve)
  Eigen::Matrix<double,Eigen::Dynamic,6> ransac_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> meas_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> alpha_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> malpha_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> ivariance_precompute_;

  // for marginalizing out previous state
  Eigen::Matrix<double, 6, 6> last_lhs_;
  Eigen::Matrix<double, 6, 1> last_rhs_;

  DOPPLER_ODOM_REGISTER_ODOMETRY("doppler_filter", DopplerFilter);
};

}  // namespace doppler_odom