#include "doppler_odom/datasets/utils.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

#include "lgmath.hpp"

namespace doppler_odom {

namespace {

double translationError(const Eigen::Matrix4d &pose_error) { return pose_error.block<3, 1>(0, 3).norm(); }

double translationError2D(const Eigen::Matrix4d &pose_error) { return pose_error.block<2, 1>(0, 3).norm(); }

double rotationError(const Eigen::Matrix4d &pose_error) {
  double a = pose_error(0, 0);
  double b = pose_error(1, 1);
  double c = pose_error(2, 2);
  double d = 0.5 * (a + b + c - 1.0);
  return std::acos(std::max(std::min(d, 1.0), -1.0));
}

double rotationError2D(Eigen::Matrix4d &pose_error) {
  auto pose_error_projected = lgmath::se3::tran2vec(pose_error);
  pose_error_projected.segment<3>(2) = Eigen::Vector3d::Zero();
  return rotationError(lgmath::se3::vec2tran(pose_error_projected));
}

std::vector<double> trajectoryDistances(const ArrayPoses &poses) {
  std::vector<double> dist(1, 0.0);
  for (size_t i = 1; i < poses.size(); i++) dist.push_back(dist[i - 1] + translationError(poses[i - 1] - poses[i]));
  return dist;
}

int lastFrameFromSegmentLength(const std::vector<double> &dist, int first_frame, double len) {
  for (int i = first_frame; i < (int)dist.size(); i++)
    if (dist[i] > dist[first_frame] + len) return i;
  return -1;
}

// void computeMeanRPE(const ArrayPoses &poses_gt, const ArrayPoses &poses_result, Sequence::SeqError &seq_err) {
//   // static parameter
//   double lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
//   size_t num_lengths = sizeof(lengths) / sizeof(double);

//   // parameters
//   int step_size = 10;  // every 10 frame (= every second for LiDAR at 10Hz)

//   // pre-compute distances (from ground truth as reference)
//   std::vector<double> dist = trajectoryDistances(poses_gt);

//   int num_total = 0;
//   double mean_t_rpe = 0;
//   double mean_t_rpe_2d = 0;
//   double mean_r_rpe = 0;
//   double mean_r_rpe_2d = 0;
//   // for all start positions do
//   for (int first_frame = 0; first_frame < (int)poses_gt.size(); first_frame += step_size) {
//     // for all segment lengths do
//     for (size_t i = 0; i < num_lengths; i++) {
//       // current length
//       double len = lengths[i];

//       // compute last frame
//       int last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

//       // next frame if sequence not long enough
//       if (last_frame == -1) continue;

//       // compute translational errors
//       Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
//       Eigen::Matrix4d pose_delta_result = poses_result[first_frame].inverse() * poses_result[last_frame];
//       Eigen::Matrix4d pose_error = pose_delta_result.inverse() * pose_delta_gt;
//       double t_err = translationError(pose_error);
//       double t_err_2d = translationError2D(pose_error);
//       double r_err = rotationError(pose_error);
//       double r_err_2d = rotationError2D(pose_error);
//       seq_err.tab_errors.emplace_back(t_err / len, r_err / len);

//       mean_t_rpe += t_err / len;
//       mean_t_rpe_2d += t_err_2d / len;
//       mean_r_rpe += r_err / len;
//       mean_r_rpe_2d += r_err_2d / len;
//       num_total++;
//     }
//   }

//   seq_err.mean_t_rpe = ((mean_t_rpe / static_cast<double>(num_total)) * 100.0);
//   seq_err.mean_t_rpe_2d = ((mean_t_rpe_2d / static_cast<double>(num_total)) * 100.0);
//   seq_err.mean_r_rpe = ((mean_r_rpe / static_cast<double>(num_total)) * 180.0 / M_PI);
//   seq_err.mean_r_rpe_2d = ((mean_r_rpe_2d / static_cast<double>(num_total)) * 180.0 / M_PI);
// }

}  // namespace

// Sequence::SeqError evaluateOdometry(const std::string &filename, const ArrayPoses &poses_gt,
//                                     const ArrayPoses &poses_est) {
//   std::ofstream errorfile(filename);
//   if (!errorfile.is_open()) throw std::runtime_error{"failed to open file: " + filename};
//   errorfile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);

//   Sequence::SeqError seq_err;

//   // Compute Mean and Max APE (Mean and Max Absolute Pose Error)
//   seq_err.mean_ape = 0.0;
//   seq_err.max_ape = 0.0;
//   for (size_t i = 0; i < poses_gt.size(); i++) {
//     double t_ape_err = translationError(poses_est[i].inverse() * poses_gt[0].inverse() * poses_gt[i]);
//     seq_err.mean_ape += t_ape_err;
//     if (seq_err.max_ape < t_ape_err) {
//       seq_err.max_ape = t_ape_err;
//     }
//   }
//   seq_err.mean_ape /= static_cast<double>(poses_gt.size());

//   // Compute Mean and Max Local Error
//   seq_err.mean_local_err = 0.0;
//   seq_err.max_local_err = 0.0;
//   seq_err.index_max_local_err = 0;
//   for (int i = 1; i < (int)poses_gt.size(); i++) {
//     Eigen::Matrix4d t_local = poses_gt[i].inverse() * poses_gt[i - 1] * poses_est[i - 1].inverse() * poses_est[i];
//     const auto t_local_vec = lgmath::se3::tran2vec(t_local);

//     double gt_local_norm_2d = (poses_gt[i].block<2, 1>(0, 3) - poses_gt[i - 1].block<2, 1>(0, 3)).norm();
//     double est_local_norm_2d = (poses_est[i].block<2, 1>(0, 3) - poses_est[i - 1].block<2, 1>(0, 3)).norm();
//     double t_local_err_2d = gt_local_norm_2d - est_local_norm_2d;

//     double gt_local_norm = (poses_gt[i].block<3, 1>(0, 3) - poses_gt[i - 1].block<3, 1>(0, 3)).norm();
//     double est_local_norm = (poses_est[i].block<3, 1>(0, 3) - poses_est[i - 1].block<3, 1>(0, 3)).norm();
//     double t_local_err = gt_local_norm - est_local_norm;

//     double abs_t_local_err = fabs(t_local_err);
//     seq_err.mean_local_err += abs_t_local_err;
//     if (seq_err.max_local_err < abs_t_local_err) {
//       seq_err.max_local_err = abs_t_local_err;
//       seq_err.index_max_local_err = i;
//     }

//     errorfile << t_local_err_2d << " " << t_local_err << " " << t_local_vec.transpose() << std::endl;
//   }
//   seq_err.mean_local_err /= static_cast<double>(poses_gt.size() - 1);

//   // Compute sequence mean RPE errors
//   computeMeanRPE(poses_gt, poses_est, seq_err);

//   return seq_err;
// }

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv) {
  std::string line;
  std::string cell;
  std::vector<std::vector<double>> mat_vec;
  while (std::getline(csv, line)) {
    std::stringstream lineStream(line);
    std::vector<double> row_vec;
    while (std::getline(lineStream, cell, ',')) {
      row_vec.push_back(std::stof(cell));
    }
    mat_vec.push_back(row_vec);
  }
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

Eigen::MatrixXd readBoreasGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro) {
  std::ifstream imu_file(file_path);
  std::vector<std::vector<double>> mat_vec;
  if (imu_file.is_open()) {
    std::string line;
    std::getline(imu_file, line);  // header
    std::vector<double> row_vec(4);
    for (; std::getline(imu_file, line);) {
      if (line.empty()) continue;
      std::stringstream ss(line);

      int64_t timestamp = 0;
      double timestamp_sec = 0;
      double r = 0, p = 0, y = 0;
      for (int i = 0; i < 4; ++i) {
        std::string value;
        std::getline(ss, value, ',');

        if (i == 0) {
          timestamp = std::stol(value);
          timestamp_sec = static_cast<double>(timestamp - initial_timestamp_micro)*1e-6;
        }
        else if (i == 1)
          r = std::stod(value);
        else if (i == 2)
          p = std::stod(value);
        else if (i == 3)
          y = std::stod(value);
      } // end for row
      // std::cout << timestamp_sec << ", " << r << ", " << p << ", " << y << std::endl;
      row_vec[0] = timestamp_sec;
      row_vec[1] = p;
      row_vec[2] = r;
      row_vec[3] = y;
      mat_vec.push_back(row_vec);
    } // end for line
  } // end if
  else {
    throw std::runtime_error{"unable to open file: " + file_path};
  }

  // output eigen matrix
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

bool filecomp (std::string file1, std::string file2) { 
  long long i = std::stoll(file1.substr(0, file1.find(".")));
  long long j = std::stoll(file2.substr(0, file2.find(".")));
  return (i<j); 
}

float atan2_approx(float y, float x) {
  static float pi = static_cast<float>(M_PI);
  static float pi_2 = static_cast<float>(M_PI_2);

  bool swap = fabs(x) < fabs(y);
  float atanin = (swap ? x : y) / (swap ? y : x);
  float a1 = 0.99997726;
  float a3 = -0.33262347;
  float a5 = 0.19354346;
  float a7 = -0.11643287;
  float a9 = 0.05265332;
  float a11 = -0.01172120;
  float atanin2 = atanin*atanin;
  float atanout = atanin * (a1 + atanin2 * (a3 + atanin2 * (a5 + atanin2 * (a7 + atanin2 * (a9 + atanin2 * a11)))));
  atanout = swap ? (atanin >= 0.0 ? pi_2 : -pi_2) - atanout : atanout;
  if (x < 0.0) {
    atanout = (y >= 0.0 ? pi : -pi) + atanout;
  }  
  return atanout;
}

}  // namespace doppler_odom