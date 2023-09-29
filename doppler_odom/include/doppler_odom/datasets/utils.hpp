#pragma once

#include <Eigen/Core>

#include "doppler_odom/dataset.hpp"

namespace doppler_odom {

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv);

Eigen::MatrixXd readBoreasGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro);

// Sequence::SeqError evaluateOdometry(const std::string &filename, const ArrayPoses &poses_gt, const ArrayPoses &poses_estimated);

}  // namespace doppler_odom