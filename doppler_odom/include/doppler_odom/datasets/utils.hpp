#pragma once

#include <Eigen/Core>
#include <vector>
#include "lgmath.hpp"

namespace doppler_odom {

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv);

Eigen::MatrixXd readGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro, const std::string& dataset);

bool filecomp(std::string file1, std::string file2);

float atan2_approx(float y, float x);

struct SensorCalib {
  // extrinsic
  std::vector<Eigen::Matrix4d> T_sv;
  std::vector<Eigen::Matrix<double,3,6>> adT_sv_top3rows;

  // gyro inverse covariance
  std::vector<Eigen::Matrix3d> gyro_invcov;
};

}  // namespace doppler_odom