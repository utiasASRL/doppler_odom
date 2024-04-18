#include "doppler_odom/odometry/doppler_filter.hpp"
#include "doppler_odom/map.hpp"

#include <iomanip>
#include <random>
#include <algorithm>
#include <glog/logging.h>
#include <iostream>

#include "doppler_odom/utils/stopwatch.hpp"
#include "lgmath.hpp"

namespace doppler_odom {

DopplerFilter::DopplerFilter(const Options &options) : Odometry(options), options_(options) {

  // extrinsics
  // TODO: move to data class and set as parameters
  T_sv_.resize(options_.num_sensors);
  T_sv_[0] << 0.9999366830849237, 0.008341717781538466, 0.0075534496251198685, -1.0119098938516395,
              -0.008341717774127972, 0.9999652112886684, -3.150635091210066e-05, -0.39658824335171944,
              -0.007553449599178521, -3.1504388681967066e-05, 0.9999714717963843, -1.697000000000001,
               0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;

  // adjoint
  adT_sv_top3rows_.resize(options_.num_sensors);
  adT_sv_top3rows_[0] = lgmath::se3::tranAd(T_sv_[0]).topRows<3>();

  // ransac generator
  //  seed_ = static_cast<long int>(std::time(nullptr)); // comment out for reproducibility
  random_engine_ = std::mt19937_64(size_t(seed_)); // Argument really should be a size type.

  // precompute wnoa lhs
  Eigen::Matrix<double, 12, 6> temp;
  temp.topRows<6>() = -Eigen::Matrix<double, 6, 6>::Identity();
  temp.bottomRows<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  wnoa_lhs_ = temp * options_.Qkinv * temp.transpose();

  // TODO: move to data class and set as parameters
  // gyro noise
  gyro_invcov_.resize(options_.num_sensors);
  gyro_invcov_[0] = Eigen::Matrix3d::Identity();
  gyro_invcov_[0](0,0) = 1.0/(2.9e-4);
  gyro_invcov_[0](1,1) = 1.0/(4.7e-4);
  gyro_invcov_[0](2,2) = 1.0/(3.4e-5);
}

DopplerFilter::~DopplerFilter() {
  std::ofstream trajectory_file;
  trajectory_file.open(options_.debug_path + "/velocity.txt", std::ios::out);
  trajectory_file << std::fixed << std::setprecision(12) << trajectory_[0].begin_timestamp << " " << 0.0 << " " << 0.0 << " " 
    << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
  for (int i = 0; i < trajectory_.size(); ++i) {
    trajectory_file << std::fixed << std::setprecision(12) << trajectory_[i].end_timestamp << " " << trajectory_[i].varpi(0) 
      << " " << trajectory_[i].varpi(1) << " " << trajectory_[i].varpi(2)
      << " " << trajectory_[i].varpi(3) << " " << trajectory_[i].varpi(4) 
      << " " << trajectory_[i].varpi(5) << std::endl;
  }

  std::ofstream pose_file;
  pose_file.open(options_.debug_path + "/pose.txt", std::ios::out);
  for (int i = 0; i < poses_.size(); ++i) {
    pose_file << std::fixed << std::setprecision(12) 
             << poses_[i](0,0) << " " << poses_[i](0,1) << " " << poses_[i](0,2) << " " << poses_[i](0,3)
      << " " << poses_[i](1,0) << " " << poses_[i](1,1) << " " << poses_[i](1,2) << " " << poses_[i](1,3)
      << " " << poses_[i](2,0) << " " << poses_[i](2,1) << " " << poses_[i](2,2) << " " << poses_[i](2,3)
      << " " << poses_[i](3,0) << " " << poses_[i](3,1) << " " << poses_[i](3,2) << " " << poses_[i](3,3) << std::endl;
  }
  LOG(INFO) << "Dumping trajectory. - DONE" << std::endl;
}

Pointcloud DopplerFilter::preprocessFrame(Pointcloud &frame, const double& start_time, const double& end_time) {
  // add a new frame
  int index_frame = trajectory_.size();
  trajectory_.emplace_back();

  // timestamps
  trajectory_[index_frame].begin_timestamp = start_time;
  trajectory_[index_frame].end_timestamp = end_time;
  LOG(INFO) << trajectory_.back().begin_timestamp << ", " <<  trajectory_.back().end_timestamp << std::endl;

  // downsample
  Pointcloud keypoint_frame = calib_->calib_frame(frame, options_.min_dist, 150.0);  // downsamples into image and runs regression

  return keypoint_frame;
}

Pointcloud DopplerFilter::ransacFrame(const Pointcloud &const_frame) {

  // initialize precomputation variables
  Eigen::Matrix<double, Eigen::Dynamic,6> ransac_precompute_all = Eigen::Matrix<double, Eigen::Dynamic, 6>(const_frame.size(), 6);
  Eigen::Matrix<double, Eigen::Dynamic,1> meas_precompute_all = Eigen::Matrix<double, Eigen::Dynamic, 1>(const_frame.size());

  // loop over each point to precompute
  for (int i = 0; i < const_frame.size(); ++i) {
    // the 'C' in y = C*x
    ransac_precompute_all.row(i) = const_frame[i].pt.transpose()/const_frame[i].range * adT_sv_top3rows_[const_frame[i].sensor_id];  

    // the 'y' in y = C*x
    meas_precompute_all(i) = const_frame[i].radial_velocity;  // the 'y' in y = C*x
  }

  // initialize uniform distribution
  std::uniform_int_distribution<int> uni_dist(0, const_frame.size() - 1);

  // ransac
  Eigen::Matrix3d lhs;
  Eigen::Vector3d rhs;
  Eigen::Vector3d G;
  int max_inliers = 0;
  // using bool_vec = Eigen::Array<bool,Eigen::Dynamic,1>;
  Eigen::Array<bool, Eigen::Dynamic,1> inliers;       
  Eigen::Array<bool, Eigen::Dynamic,1> best_inliers;  
  Eigen::Vector3d best_varpi;   // for debugging
  Eigen::Vector3d curr_varpi;   // for debugging
  for (int iter = 0; iter < options_.ransac_max_iter; ++iter) {
    // setup linear system
    lhs.setZero();
    rhs.setZero();

    // sample until we satisfy min. range condition for ransac
    int sample1, sample2;
    for (int k = 0; k < 1e3; ++k) { // 1e3 is safety measure to prevent infinite loop
      sample1 = uni_dist(random_engine_);
      if (const_frame[sample1].range > options_.ransac_min_range)
        break;
    }
    int safety_count = 0;
    for (int k = 0; k < 1e3; ++k) { // 1e3 is safety measure to prevent infinite loop
      sample2 = uni_dist(random_engine_);
      if (const_frame[sample2].range > options_.ransac_min_range)
        break;
    }

    // sample 1
    G(0) = ransac_precompute_all(sample1, 0);
    G(1) = ransac_precompute_all(sample1, 1);
    G(2) = ransac_precompute_all(sample1, 5);
    lhs += G * G.transpose();
    rhs += G * meas_precompute_all(sample1);

    // sample 2
    G(0) = ransac_precompute_all(sample2, 0);
    G(1) = ransac_precompute_all(sample2, 1);
    G(2) = ransac_precompute_all(sample2, 5);
    lhs += G * G.transpose();
    rhs += G * meas_precompute_all(sample2);

    // 2 DOF solve
    Eigen::Matrix2d lhs2d;
    lhs2d << lhs(0, 0), lhs(0, 2), lhs(2, 0), lhs(2, 2);
    if (fabs(lhs2d.determinant()) < 1e-7)
      continue; // not invertible

    Eigen::Vector2d rhs2d;
    rhs2d << rhs(0), rhs(2);

    Eigen::Vector2d varpi2d = lhs2d.inverse()*rhs2d;  // inverse should be fast for 2x2
    curr_varpi << varpi2d(0), 0.0, varpi2d(1);

    // calculate error and inliers 
    auto errors = meas_precompute_all - ransac_precompute_all.col(0)*curr_varpi(0) 
                                      - ransac_precompute_all.col(1)*curr_varpi(1) 
                                      - ransac_precompute_all.col(5)*curr_varpi(2);
    inliers = errors.array().abs() < options_.ransac_thres;
    int num_inliers = inliers.count();

    // check for improvement in number of inliers
    if (num_inliers > max_inliers) {
      max_inliers = num_inliers;
      best_varpi = curr_varpi;  // for debugging
      best_inliers = inliers;
    }
  }
  // std::cout << best_varpi.transpose() << std::endl;   // for debugging

  // create new frame with only inliers
  Pointcloud inlier_frame;
  inlier_frame.reserve(max_inliers);

  // precompute for full solve
  ransac_precompute_ = Eigen::Matrix<double, Eigen::Dynamic, 6>(max_inliers, 6);
  meas_precompute_ = Eigen::Matrix<double, Eigen::Dynamic, 1>(max_inliers);
  alpha_precompute_ = Eigen::Matrix<double, Eigen::Dynamic, 1>(max_inliers);
  malpha_precompute_ = Eigen::Matrix<double, Eigen::Dynamic, 1>(max_inliers);

  // loop over each measurement
  int k = 0;
  for (int i = 0; i < const_frame.size(); ++i) {
    if (!best_inliers(i))
      continue; // skip since it's not an inlier

    inlier_frame.push_back(const_frame[i]);
    ransac_precompute_.row(k) = ransac_precompute_all.row(i);
    meas_precompute_[k] = meas_precompute_all[i];
    alpha_precompute_[k] = std::min(1.0, std::max(0.0, (const_frame[i].timestamp - trajectory_.back().begin_timestamp) / 
        (trajectory_.back().end_timestamp - trajectory_.back().begin_timestamp)));
    malpha_precompute_[k] = std::max(0.0, 1.0 - alpha_precompute_[k]);
    ++k;
  }

  LOG(INFO) << "before ransac: " << const_frame.size() << std::endl;
  LOG(INFO) << "after ransac: " << inlier_frame.size() << std::endl;
  return inlier_frame;
}

void DopplerFilter::solveFrame(const Pointcloud &const_frame, const std::vector<Eigen::MatrixXd> &gyro) {
  // we build a 12x12 linear system and marginalize to a 6x6 system to solve for the latest vehicle velocity
  Eigen::Matrix<double, 12, 12> lhs = Eigen::Matrix<double, 12, 12>::Zero();
  Eigen::Matrix<double, 12, 1> rhs = Eigen::Matrix<double, 12, 1>::Zero();

  if (trajectory_.size() == 1) {
    // prior
    lhs.topLeftCorner<6, 6>() += options_.P0inv;
  }
  else {
    lhs.topLeftCorner<6, 6>() += last_lhs_;
    rhs.topLeftCorner<6, 1>() += last_rhs_;
  }

  // wnoa prior
  lhs += wnoa_lhs_;

  // zero velocity prior
  lhs.bottomRightCorner<6, 6>() += options_.Qzinv;

  // IMU measurements
  for (int i = 0; i < options_.num_sensors; ++i) {
    if (gyro[i].rows() == 1 && gyro[i].cols() == 1)
      continue; // no data

    Eigen::Matrix<double,3,6> Cgyro = Eigen::Matrix<double, 3, 6>::Zero();
    Cgyro.rightCols<3>() = T_sv_[i].topLeftCorner<3, 3>();
    Eigen::Matrix<double,3,12> Ggyro = Eigen::Matrix<double, 3, 12>::Zero();

    // loop over each gyro measurement
    for (int j = 0; j < gyro[i].rows(); ++j) {
      double alpha = std::min(1.0, std::max(0.0, (gyro[i](j, 0) - trajectory_.back().begin_timestamp)
          /(trajectory_.back().end_timestamp - trajectory_.back().begin_timestamp)));
      Ggyro.leftCols<6>() = (1.0 - alpha)*Cgyro;
      Ggyro.rightCols<6>() = alpha*Cgyro;

      lhs += Ggyro.transpose() * gyro_invcov_[i] * Ggyro;
      rhs += Ggyro.transpose() * gyro_invcov_[i] * (gyro[i].row(j).rightCols<3>().transpose() - options_.const_gyro_bias[i]);
    }
  } // end for i

  // doppler measurements
  Eigen::Matrix<double, Eigen::Dynamic, 12> G(const_frame.size(), 12); // N x 12
  G.leftCols<6>() = ransac_precompute_.array().colwise() * malpha_precompute_.array();
  G.rightCols<6>() = ransac_precompute_.array().colwise() * alpha_precompute_.array();
  lhs += G.transpose() * G / (0.2*0.2);   // TODO: add variance as parameter
  rhs += G.transpose() * meas_precompute_ / (0.2*0.2);

  // marginalize
  Eigen::Matrix<double, 6, 6> temp = lhs.bottomLeftCorner<6,6>()*lhs.topLeftCorner<6,6>().inverse();
  Eigen::Matrix<double, 6, 6> lhs_new = lhs.bottomRightCorner<6,6>() - temp*lhs.topRightCorner<6,6>();
  Eigen::Matrix<double, 6, 1> rhs_new = rhs.tail<6>() - temp*rhs.head<6>();

  // solve
  trajectory_.back().varpi = lhs_new.llt().solve(rhs_new);
  last_lhs_ = lhs_new;
  last_rhs_ = rhs_new;

  // std::cout << trajectory_.back().varpi.transpose() << std::endl;
  return;
}

void DopplerFilter::initializeTimestamp(int index_frame, const std::vector<Pointcloud> &const_frames) {
  double min_timestamp = std::numeric_limits<double>::max();
  double max_timestamp = std::numeric_limits<double>::min();
  for (const auto &const_frame : const_frames) {
    for (const auto &point : const_frame) {
      if (point.timestamp > max_timestamp) max_timestamp = point.timestamp;
      if (point.timestamp < min_timestamp) min_timestamp = point.timestamp;
    }
  }
  trajectory_[index_frame].begin_timestamp = min_timestamp;
  trajectory_[index_frame].end_timestamp = max_timestamp;
}

Eigen::Matrix4d DopplerFilter::integrateForPose() {
  // get velocity knots
  Eigen::Matrix<double,6,1> knot1 = Eigen::Matrix<double,6,1>::Zero();
  double dt = 0.1;
  if (trajectory_.size() > 1) {
    knot1 = trajectory_[trajectory_.size()-2].varpi;
    dt = trajectory_.back().end_timestamp - trajectory_[trajectory_.size()-2].end_timestamp;
  }
  Eigen::Matrix<double,6,1> knot2 = trajectory_.back().varpi;

  // mask to zero when stationary
  if (std::fabs(knot1(0)) < options_.zero_vel_tol)
    knot1 = Eigen::Matrix<double,6,1>::Zero();
  if (std::fabs(knot2(0)) < options_.zero_vel_tol)
    knot2 = Eigen::Matrix<double,6,1>::Zero();

  // integrate between the knots
  double dtt = dt/static_cast<double>(options_.integration_steps);
  Eigen::Matrix4d T_21 = Eigen::Matrix4d::Identity();
  for (int s = 1; s <= options_.integration_steps; ++s) {
    double t = s*dtt;
    double alpha = t/dt;
    Eigen::Matrix<double,6,1> vinterp = (1.0-alpha)*knot1 + alpha*knot2;
    T_21 = lgmath::se3::vec2tran(dtt*vinterp)*T_21;
  }

  // output latest pose
  if (poses_.size() == 0) 
    poses_.push_back(Eigen::Matrix4d::Identity());
  poses_.push_back(T_21*poses_.back());
  return T_21;
}

}  // namespace doppler_odom
