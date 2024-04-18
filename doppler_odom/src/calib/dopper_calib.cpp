#include "doppler_odom/calib/doppler_calib.hpp"
#include "doppler_odom/datasets/utils.hpp"

#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <unordered_map>

namespace doppler_odom {

float atan2_approx(float y, float x, float pi, float pi_2) {
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

DopplerCalib::DopplerCalib(const Options& options) : options_(options) {
  int num_sensors = options.active_sensors.size();

  // read elevation settings
  elevation_order_.clear();
  elevation_order_by_beam_id_.clear();
  for (int i = 0; i < num_sensors; ++i) {
    if (!options.active_sensors[i])
      continue;

    std::string path = options.root_path + "/mean_elevation_beam_order_" + std::to_string(i);
    std::ifstream csv(path);
    if (!csv) throw std::ios::failure("Error opening csv file");
    elevation_order_.push_back(readCSVtoEigenXd(csv));
    const auto& temp = elevation_order_.back();

    std::vector<Eigen::MatrixXd> sensor_elevation_order;
    for (int j = 0; j < 4; ++j) {   // 4 beams   
      Eigen::MatrixXd elevation_order_for_this_beam(temp.rows()/4, 2);  // first column is mean elevation, second column is row id
      int h = 0;
      for (int r = 0; r < temp.rows(); ++r) {
        // first column is mean elevation. Second column is beam id
        if (temp(r, 1) == j) {
          elevation_order_for_this_beam(h, 0) = temp(r, 0);
          elevation_order_for_this_beam(h, 1) = r;
          ++h;
        }
      } // end for r
      assert(h == temp.rows()/4);
      sensor_elevation_order.push_back(elevation_order_for_this_beam);
    } // end for j
    assert(sensor_elevation_order.size() == 4); // 4 beams
    elevation_order_by_beam_id_.push_back(sensor_elevation_order);
  } // end for i

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

std::vector<Point3D> DopplerCalib::calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const {
  // required for atan2 approximation
  float pi = M_PI;
  float pi_2 = M_PI_2;

  // 2D vector of pointers to points
  using PointWFlag = std::pair<bool,const Point3D*>;
  int grid_count = 0;
  std::vector<std::vector<PointWFlag>> grid(options_.num_rows, std::vector<PointWFlag>(options_.num_cols, PointWFlag(false, NULL)));
  // iterate over each point
  for (auto& point : frame) {
    point.range = sqrt(point.pt[0]*point.pt[0] + point.pt[1]*point.pt[1] + point.pt[2]*point.pt[2]);
    if (point.range < min_dist || point.range > max_dist)
      continue;

    // polynomial approx. of atan2
    // double azimuth = atan2(point.pt[1], point.pt[0]);
    const double azimuth = atan2_approx(point.pt[1], point.pt[0], pi, pi_2);
    const double xy = sqrt(point.pt[0]*point.pt[0] + point.pt[1]*point.pt[1]);
    // double elevation = atan2(point.pt[2], xy);
    const double elevation = atan2_approx(point.pt[2], xy, pi, pi_2);

    // skip if not within azimuth bounds (horizontal fov)
    if (azimuth <= options_.azimuth_start || azimuth >= options_.azimuth_end)
      continue;

    // determine column
    const short col = (options_.num_cols - 1) - int((azimuth - options_.azimuth_start)/options_.azimuth_res);
    
    // determine row by matching by beam_id (0, 1, 2, or 3) and closest elevation to precalculated values
    // note: elevation_order_by_beam_id_[sensorid][point.beam_id] first column is mean elevation, second column is row id
    const auto ele_diff = elevation_order_by_beam_id_[point.sensor_id][point.beam_id].col(0).array() - elevation;
    double min_val = ele_diff(0)*ele_diff(0);
    int min_id = 0;
    for (int i = 1; i < ele_diff.rows(); ++i) {
      const auto val = ele_diff(i) * ele_diff(i);
      if (val < min_val) {
        min_val = val;
        min_id = i;
      }
    }
    
    // picking the closest in elevation
    const short row = elevation_order_by_beam_id_[point.sensor_id][point.beam_id](min_id, 1);
    if (!grid[row][col].first) {
      // keep first measurement in bin
      grid[row][col] = PointWFlag(true, &point);
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