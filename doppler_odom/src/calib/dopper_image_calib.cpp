#include "doppler_odom/calib/doppler_image_calib.hpp"
#include "doppler_odom/datasets/utils.hpp"

#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <unordered_map>

namespace doppler_odom {

DopplerImageCalib::DopplerImageCalib(const Options& options) : options_(options) {
  // init weights
  initImgWeight(true, "bias_shape.txt", "bias.bin", bias_weights_);
  initImgWeight(false, "var_shape.txt", "var.bin", var_weights_);

  // read yaml config
  std::string config_path = options_.root_path + "/" + options_.model_name + "/config.yaml";
  YAML::Node config = YAML::LoadFile(config_path);
  bias_features_ = config["train"]["bias_net"]["bias_features"]["input_features"].as<std::vector<std::string>>();
  var_features_ = config["train"]["bias_net"]["var_features"]["input_features"].as<std::vector<std::string>>();
  bias_porder_ = config["train"]["bias_net"]["bias_features"]["polyorder"].as<int>();
  var_porder_ = config["train"]["bias_net"]["var_features"]["polyorder"].as<int>();

  // check if we need to calculate median Doppler velocity
  for (const auto& feat: bias_features_)
    if (feat == "medianrv")
      calc_dop_median_ = true;
  for (const auto& feat: var_features_) {
    if (feat == "medianrv")
      calc_dop_median_ = true;
    if (feat == "rv_var5")  // TODO: handle variable parameter in string
      calc_pseudo_var_ = true;
  }

  // TODO: check for rv_stddev5
}

void DopplerImageCalib::initImgWeight(bool set_dims, const std::string& dim_txt, const std::string& binary, std::vector<std::vector<ImgWeight>>& weights) {
  // read csv that specifies dimensions
  std::ifstream csv(options_.root_path + "/" + options_.model_name + "/" + dim_txt);
  Eigen::MatrixXi dims = readCSVtoEigenXd(csv).cast<int>();  // 0(# sensors) x 1(# rows) x 2(# cols) x 3(# faces) x 4(weight dim)

  // reassigns values for rows and cols based on weights data
  if (set_dims) {
    options_.num_rows = dims(1);
    options_.num_cols = dims(2);
  }

  Eigen::VectorXd dummy_vec(dims(4)); // dummy vector with appropriate size
  ImgWeight bias_weight(dims(1), std::vector<Eigen::VectorXd>(dims(2), dummy_vec)); // (# rows) x (# cols) x (weight dim)

  // initialize with approriate (# sensors) x (# faces)
  weights = std::vector<std::vector<ImgWeight>>(dims(0), std::vector<ImgWeight>(dims(3), bias_weight));

  // read binary
  std::ifstream ifs(options_.root_path + "/" + options_.model_name + "/" + binary, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4;
  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float { return *((float *)(byteArray + index)); };

  for (size_t sensor = 0; sensor < dims(0); ++sensor) {
    for (size_t row = 0; row < dims(1); ++row) {
      for (size_t col = 0; col < dims(2); ++col) {
        for (size_t face = 0; face < dims(3); ++face) {
          for (size_t d = 0; d < dims(4); ++d) {  
            int offset = d + dims(4)*face + dims(4)*dims(3)*col 
              + dims(4)*dims(3)*dims(2)*row + dims(4)*dims(3)*dims(2)*dims(1)*sensor;
            weights[sensor][face][row][col](d) = getFloatFromByteArray(buffer.data(), offset * float_offset);
          } // d
        } // face
      } // col
    } // row
  } // sensor
}

std::vector<Point3D> DopplerImageCalib::calib_frame(std::vector<Point3D> &frame, const double& min_dist, const double& max_dist) const {
  // 2D vector of pointers to points
  // using PointWFlag = std::pair<bool,const Point3D*>;
  using PointImg = std::vector<std::vector<const Point3D*>>;
  // using PointImg = std::unordered_map<int, const Point3D*>;
  
  // initialize empty grid (2D img filled with null pointers)
  PointImg empty_img(options_.num_rows, std::vector<const Point3D*>(options_.num_cols, nullptr));
  // PointImg empty_img(options_.num_rows * options_.num_cols);

  // create an image for each sensor
  int num_sensors = bias_weights_.size();
  std::vector<PointImg> imgs(num_sensors, empty_img);
  // int pt_count = 0;  // keeps track of total # points to reserve later

  // doppler median
  // std::vector<std::vector<double>> dop_vels;
  std::vector<double> dop_vels;
  // if (calc_dop_median_) {
  //   // note: not reserving in advance seems to be better since we have to set an upper-bound on # points
  //   // dop_vels = std::vector<std::vector<double>>(num_sensors, std::vector<double>()); // 1 for each sensor
  // }

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
    if (col < 0 || col >= imgs[point.sensor_id][point.line_id].size())
      continue;
    
    // picking the closest in elevation
    if (imgs[point.sensor_id][point.line_id][col] == nullptr) {
      // keep first measurement in bin
      imgs[point.sensor_id][point.line_id][col] = &point;
      // ++pt_count;

      // stack velocities for median calculation
      if (calc_dop_median_ && point.sensor_id == 0) { // TODO: change sensor id for aevaHQ
        // dop_vels[point.sensor_id].push_back(point.radial_velocity);
        dop_vels.push_back(point.radial_velocity);
      }
    }

    // keep first measurement in bin
    // imgs[point.sensor_id][point.line_id*options_.num_cols + col] = &point;
    // auto flag = imgs[point.sensor_id].emplace(point.line_id*options_.num_cols + col, &point);
    // if (flag.second)
    //   ++pt_count;
  }

  // median calculation
  // TODO: for multiple sensors, we compute only for 1 (set as parameter)
  // no need for vector of doppler median values
  double dop_median;
  if (calc_dop_median_) {
    int n = dop_vels.size()/2;
    auto nitr = dop_vels.begin() + n;
    std::nth_element(dop_vels.begin(), nitr, dop_vels.end());
    dop_median = *nitr;
  }

  // output
  std::vector<Point3D> out_frame;
  // out_frame.reserve(pt_count);
  out_frame.reserve(80 * 500);  // TODO: update
  Eigen::VectorXd bias_feat(bias_features_.size());
  Eigen::VectorXd var_feat(var_features_.size());
  int dscount = 0;
  for (size_t s = 0; s < num_sensors; ++s) {
    for (size_t r = 0; r < options_.num_rows; ++r) {
      for (size_t c = 0; c < options_.num_cols; ++c) {
        if (imgs[s][r][c] != nullptr) {

          // step downsample after image projection
          ++dscount;
          if (dscount % options_.downsample_steps != 0)
            continue;

          // pseudo-variance
          double pseudovar = 1.0;
          if (calc_pseudo_var_) {
            bool varflag = computePseudovar(pseudovar, imgs[s][r], c, pseudo_var_hwidth_, 9999);
            if (!varflag)
              continue;
          }

          // pushback if we have data in this elevation-azimuth bin
          out_frame.push_back(*imgs[s][r][c]);
          
          // build features
          buildFeatVec(bias_feat, out_frame.back(), bias_features_, dop_median, pseudovar);
          buildFeatVec(var_feat, out_frame.back(), var_features_, dop_median, pseudovar);

          // apply linear regression model
          // out_frame.back().radial_velocity -= (weights_[sensorid][r](c, 0) + weights_[sensorid][r](c, 1)*out_frame.back().range/250.0);
          out_frame.back().radial_velocity -= computeModel(bias_feat, bias_weights_[s][out_frame.back().face_id][r][c], bias_porder_);
          out_frame.back().ivariance = exp(computeModel(var_feat, var_weights_[s][out_frame.back().face_id][0][0], var_porder_));
          // out_frame.back().ivariance = exp(computeModel(var_feat, var_weights_[0][0][0][0], var_porder_));
        }
      }
    }
  }
  // for (size_t s = 0; s < num_sensors; ++s) {
  //   for (const auto& point: imgs[s]) {
  //     out_frame.push_back(*point.second);
  //   }
  // }
  return out_frame;
}

void DopplerImageCalib::buildFeatVec(Eigen::VectorXd& feat, const Point3D& point, 
    const std::vector<std::string>& feat_string, double dop_median, double dop_pseudovar) const {

  auto scaleValue = [](double value, double lower, double upper) -> double { return 2.0*(value - lower)/(upper - lower) - 1.0; }; 

  for (size_t i = 0; i < feat_string.size(); ++i) {
    double val = 0;
    if (feat_string[i] == "range")
      val = scaleValue(point.range, 0.0, 150.0);  // note: range is calculated in preprocess function of DopplerFilter
    else if (feat_string[i] == "intensity")
      val = scaleValue(point.intensity, -70.0, 0.0);
    else if (feat_string[i] == "medianrv")
      val = scaleValue(dop_median, -30.0, 1.0); // TODO: scales different if back facing...
    else if (feat_string[i] == "rv_var5") // TODO: handle variable
      val = scaleValue(std::min(1.0 / sqrt(dop_pseudovar), 200.0), 0.0, 200.0);
    else
      throw std::runtime_error("[DopplerImageCalib::buildFeatVec] Unknown feature!");
    feat(i) = val;  // set value
  }
}

double DopplerImageCalib::computeModel(const Eigen::VectorXd& feat, const Eigen::VectorXd& weights, int polyorder) const {
  if (polyorder * feat.size() + 1 != weights.size()) {
    LOG(WARNING) << "[DopplerImageCalib::computeModel] Incompatible feature and weight dimensions!" 
                 << polyorder << ", " << feat.size() << ", " << weights.size() << std::endl;
    throw std::runtime_error("[DopplerImageCalib::computeModel] Incompatible feature and weight dimensions!");
  }
  
  double output = 0;
  // Eigen::VectorXd featpow = feat;
  Eigen::VectorXd featpow = Eigen::VectorXd::Ones(feat.size()); // TODO: bug in training code that starts with feat ^ 0 = 1
  for (int i = 0; i < polyorder; ++i) {
    output += featpow.dot(weights.segment(i * feat.size(), feat.size()));
    featpow.array() *= feat.array();
  }
  output += weights(weights.size() - 1);  // bias term
  return output;
}

bool DopplerImageCalib::computePseudovar(double& pseudovar, const std::vector<const Point3D*>& img_row, int c, int hwidth, double tol) const {
  pseudovar = 1.0;  // default value
  double vbar = img_row[c]->radial_velocity;  // use value at (r, c) as the "mean"
  int count = 0;  // count for calculating variance
  double sum = 0; // sum for calculating variance

  // loop from -hwidth to +hwidth
  for (int ci = c - hwidth; ci <= c + hwidth; ++ci) {
    if (ci == c || ci < 0 || ci >= options_.num_cols || img_row[ci] == nullptr)
      continue; // skip if ci is at c, or no measurement at (r, ci), or beyond limits

    double diff = img_row[ci]->radial_velocity - vbar;
    if (fabs(diff) > tol)
      continue; // skip if value difference is beyond tolerance

    // okay to add
    sum += (diff * diff);
    ++count;
  } // end for ci

  if (count > 3 && sum != 0) {
    pseudovar = sum/count; // set pseudovariance
    return true;
  }
  else {
    return false;     // skip this measurement since there are no neighbours
  }
}

} // namespace