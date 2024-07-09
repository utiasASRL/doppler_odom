#include "doppler_odom/datasets/utils.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace doppler_odom {

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

Eigen::MatrixXd readGyroToEigenXd(const std::string &file_path, const int64_t& initial_timestamp_micro, const std::string& dataset) {
  // this function is specifically designed for 2 datasets: boreas_aeva and aevahq
  if (dataset != "boreas_aeva" && dataset != "aevahq") 
    throw std::runtime_error{"[readGyroToEigenXd] unknown dataset specified!"};

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
      for (int i = 0; i < 7; ++i) {
        std::string value;
        std::getline(ss, value, ',');

        if (i == 0) {
          timestamp = std::stol(value);
          timestamp_sec = static_cast<double>(timestamp - initial_timestamp_micro)*1e-6;
        }
        else if (dataset == "boreas_aeva") {  // Note: r and p are flipped for boreas_aeva
          if (i == 2) 
            r = std::stod(value);
          else if (i == 1)
            p = std::stod(value);
          else if (i == 3)
            y = std::stod(value);
        }
        else /* if (dataset == "aevahq") */ {
          if (i == 4)
            r = std::stod(value);
          else if (i == 5)
            p = std::stod(value);
          else if (i == 6)
            y = std::stod(value);
        }
      } // end for row
      // std::cout << timestamp_sec << ", " << r << ", " << p << ", " << y << std::endl;
      row_vec[0] = timestamp_sec;
      row_vec[1] = r;
      row_vec[2] = p;
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