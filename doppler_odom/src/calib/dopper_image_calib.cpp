#include "doppler_odom/calib/doppler_image_calib.hpp"
#include "doppler_odom/datasets/utils.hpp"

#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <unordered_map>

namespace doppler_odom {

DopplerImageCalib::DopplerImageCalib(const Options& options) : options_(options) {
  int num_sensors = options.active_sensors.size();

  // TODO: handle different regression models. Load right model according to a parameter (currently hardcoded)
  weights_.clear();
  for (int i = 0; i < num_sensors; ++i) {
    if (!options.active_sensors[i])
      continue;

    std::vector<Eigen::MatrixXd> temp; temp.clear();
    for (int r = 0; r < options.num_rows; ++r) {
      std::string path = options.root_path + "/sensor" + std::to_string(i) + "_s1/lr_weights_row_" + std::to_string(r);
      
      std::ifstream csv(path);
      if (!csv) throw std::ios::failure("Error opening csv file");
      Eigen::MatrixXd dummy = readCSVtoEigenXd(csv);
      temp.push_back(dummy);
    }
    weights_.push_back(temp);
  }
}

std::vector<Point3D> DopplerImageCalib::calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const {
  // 2D vector of pointers to points
  using PointWFlag = std::pair<bool,const Point3D*>;
  int grid_count = 0;
  std::vector<std::vector<PointWFlag>> grid(options_.num_rows, std::vector<PointWFlag>(options_.num_cols, PointWFlag(false, NULL)));
  // iterate over each point
  for (auto& point : frame) {
    // polynomial approx. of atan2
    // double azimuth = atan2(point.pt[1], point.pt[0]);
    const double azimuth = atan2_approx(point.pt[1], point.pt[0]);

    // skip if not within azimuth bounds (horizontal fov)
    if (azimuth <= options_.azimuth_start || azimuth >= options_.azimuth_end)
      continue;

    // determine column
    const short col = (options_.num_cols - 1) - int((azimuth - options_.azimuth_start)/options_.azimuth_res);
    
    // picking the closest in elevation
    if (!grid[point.line_id][col].first) {
      // keep first measurement in bin
      grid[point.line_id][col] = PointWFlag(true, &point);
      ++grid_count;
    }
  }

  // output
  std::vector<Point3D> out_frame;
  out_frame.reserve(grid_count);
  for (int r = 0; r < options_.num_rows; ++r) {
    for (int c = 0; c < options_.num_cols; ++c) {
      if (grid[r][c].first) {
        // pushback if we have data in this elevation-azimuth bin
        out_frame.push_back(*grid[r][c].second);
        int sensorid = (*grid[r][c].second).sensor_id;

        // apply linear regression model
        out_frame.back().radial_velocity -= (weights_[sensorid][r](c, 0) + weights_[sensorid][r](c, 1)*out_frame.back().range/250.0);
      }
    }
  }

  return out_frame;
}

} // namespace