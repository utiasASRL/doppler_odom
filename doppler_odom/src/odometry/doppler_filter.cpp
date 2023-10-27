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

  // TODO: set as parameters
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

std::vector<Pointcloud> DopplerFilter::preprocessFrame(std::vector<Pointcloud> &frames, const double& start_time, const double& end_time) {
  // add a new frame
  int index_frame = trajectory_.size();
  trajectory_.emplace_back();

  // timestamps
  trajectory_[index_frame].begin_timestamp = start_time;
  trajectory_[index_frame].end_timestamp = end_time;
  LOG(INFO) << trajectory_.back().begin_timestamp << ", " <<  trajectory_.back().end_timestamp << std::endl;

  // downsample
  std::vector<Pointcloud> keypoint_frames;
  int sensorid = 0;
  for (auto &frame : frames) {
    if (frame.size() > 0) {
      // randomly shuffle
      // std::vector<size_t> v(frame.size());
      // std::iota(v.begin(), v.end(), 0);
      // std::shuffle(std::begin(v), std::end(v), random_engine_);  //  slow, adds ~2ms
      // auto keypoints = calib_->calib_frame(frame, sensorid, 20.0, 150.0);  // downsamples into image and runs regression
      auto keypoints = calib_->calib_frame(frame, sensorid, options_.min_dist, 150.0);  // downsamples into image and runs regression
      keypoint_frames.push_back(keypoints);
    }
    else
      keypoint_frames.push_back(Pointcloud());  // empty pointcloud
    ++sensorid;
  }

  assert(keypoint_frames.size() == options_.num_sensors);
  return keypoint_frames;
}

std::vector<Pointcloud> DopplerFilter::ransacFrame(const std::vector<Pointcloud> &const_frames) {
  // these matrices/vectors are precomputed so we don't repeat calculations in ransac iterations
  // also used when we estimate the 6DOF velocity in registerFrame
  ransac_precompute_.clear(); ransac_precompute_.resize(options_.num_sensors);  // reset
  meas_precompute_.clear(); meas_precompute_.resize(options_.num_sensors);      // reset
  alpha_precompute_.clear();  alpha_precompute_.resize(options_.num_sensors);   // reset
  malpha_precompute_.clear(); malpha_precompute_.resize(options_.num_sensors);  // reset

  int num_active = 0;   // number of active sensors in this frame (can vary if a sensor drops a frame)
  sensor_active_.clear();

  // These vectors will have all the measurements. Outliers will be need to be filtered out later.
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,6>> ransac_precompute_temp; ransac_precompute_temp.resize(options_.num_sensors);
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>> meas_precompute_temp; meas_precompute_temp.resize(options_.num_sensors);

  std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>> errors; errors.resize(options_.num_sensors);
  std::vector<std::uniform_int_distribution<int>> dist_vec; // uniform random distribution for each sensor

  // loop over each sensor
  for (int i = 0; i < options_.num_sensors; ++i) {
    if (const_frames[i].size() <= 0) {
      dist_vec.push_back(std::uniform_int_distribution<int>(0, 1)); // dummy distribution
      sensor_active_.push_back(false);
      continue; // no points
    }

    num_active++; // this sensor is active, so increment
    sensor_active_.push_back(true);
    ransac_precompute_temp[i] = Eigen::Matrix<double,Eigen::Dynamic,6>(const_frames[i].size(), 6);  // preallocate
    meas_precompute_temp[i] = Eigen::Matrix<double,Eigen::Dynamic,1>(const_frames[i].size());       // preallocate
    errors[i] = Eigen::Matrix<double,Eigen::Dynamic,1>(const_frames[i].size());                 // preallocate

    // loop over each point to construct 'stacked' measurement model
    for (int j = 0; j < const_frames[i].size(); ++j) {
      // ransac_precompute_temp[i].row(j) = const_frames[i][j].pt.normalized().transpose() * adT_sv_top3rows_[i];  // the 'C' in y = C*x
      ransac_precompute_temp[i].row(j) = const_frames[i][j].pt.transpose()/const_frames[i][j].range * adT_sv_top3rows_[i];  // the 'C' in y = C*x
      meas_precompute_temp[i](j) = const_frames[i][j].radial_velocity;  // the 'y' in y = C*x
    }
    dist_vec.push_back(std::uniform_int_distribution<int>(0, const_frames[i].size() - 1));  // sets index range
  }

  // ransac
  Eigen::Matrix3d lhs;
  Eigen::Vector3d rhs;
  Eigen::Vector3d G;
  int max_inliers = 0;
  using bool_vec = Eigen::Array<bool,Eigen::Dynamic,1>;
  std::vector<std::shared_ptr<bool_vec>> inliers;       
  std::vector<std::shared_ptr<bool_vec>> best_inliers;  
  inliers.resize(options_.num_sensors);       // each element corresponds to a sensor
  best_inliers.resize(options_.num_sensors);  // each element corresponds to a sensor
  Eigen::Vector3d best_varpi;   // for debugging
  Eigen::Vector3d curr_varpi;   // for debugging
  for (int iter = 0; iter < options_.ransac_max_iter; ++iter) {
    // setup linear system
    lhs.setZero();
    rhs.setZero();

    // loop over each sensor
    for (int i = 0; i < options_.num_sensors; ++i) {
      if (!sensor_active_[i])
        continue; // skip

      // int sample1 = dist_vec[i](random_engine_);
      // int sample2 = dist_vec[i](random_engine_);

      // sample until we satisfy min. range condition for ransac
      int sample1, sample2;
      while (true) {
        sample1 = dist_vec[i](random_engine_);
        if (const_frames[i][sample1].range > options_.ransac_min_range)
          break;
      }
      while (true) {
        sample2 = dist_vec[i](random_engine_);
        if (const_frames[i][sample2].range > options_.ransac_min_range)
          break;
      }

      // sample 1
      G(0) = ransac_precompute_temp[i](sample1, 0);
      G(1) = ransac_precompute_temp[i](sample1, 1);
      G(2) = ransac_precompute_temp[i](sample1, 5);
      lhs += G * G.transpose();
      rhs += G * meas_precompute_temp[i](sample1);

      // sample 2
      G(0) = ransac_precompute_temp[i](sample2, 0);
      G(1) = ransac_precompute_temp[i](sample2, 1);
      G(2) = ransac_precompute_temp[i](sample2, 5);
      lhs += G * G.transpose();
      rhs += G * meas_precompute_temp[i](sample2);
    } // end loop i

    int num_inliers = 0;
    if (num_active == 1) {
      // 2 dof (only 1 sensor available)
      Eigen::Matrix2d lhs2d;
      lhs2d << lhs(0, 0), lhs(0, 2), lhs(2, 0), lhs(2, 2);
      if (fabs(lhs2d.determinant()) < 1e-7)
        continue; // not invertible

      Eigen::Vector2d rhs2d;
      rhs2d << rhs(0), rhs(2);

      Eigen::Vector2d varpi2d = lhs2d.inverse()*rhs2d;  // inverse should be fast for 2x2
      curr_varpi << varpi2d(0), 0.0, varpi2d(1);
    }
    else if (num_active > 1) {
      // 3 dof (multiple sensors available)
      if (fabs(lhs.determinant()) < 1e-7)
        continue; // not invertible
      curr_varpi = lhs.inverse()*rhs;  // inverse should be fast for 3x3
    }
    else
      throw std::runtime_error("invalid ransac");

    // calculate error and inliers (loop over each sensor)
    for (int i = 0; i < options_.num_sensors; ++i) {
      if (!sensor_active_[i])
        continue; // skip
      errors[i] = meas_precompute_temp[i] - ransac_precompute_temp[i].col(0)*curr_varpi(0) - ransac_precompute_temp[i].col(1)*curr_varpi(1) 
        - ransac_precompute_temp[i].col(5)*curr_varpi(2);
      inliers[i] = std::make_shared<bool_vec>(errors[i].array().abs() < options_.ransac_thres);
      num_inliers += inliers[i]->count();
    } // end for i

    if (num_inliers > max_inliers) {
      max_inliers = num_inliers;
      best_varpi = curr_varpi;  // for debugging
      for (int i = 0; i < options_.num_sensors; ++i)
        best_inliers[i] = inliers[i];
    }
  }
  // std::cout << best_varpi.transpose() << std::endl;   // for debugging

  // create new frame with only inliers
  std::vector<Pointcloud> inlier_frames;  inlier_frames.resize(options_.num_sensors); // preallocate

  // loop over each sensor
  for (int i = 0; i < options_.num_sensors; ++i) {
    if (!sensor_active_[i])
      continue; // no points. skip
    int num_inliers = best_inliers[i]->count();
    inlier_frames[i].reserve(num_inliers);
    ransac_precompute_[i] = Eigen::Matrix<double,Eigen::Dynamic,6>(num_inliers, 6);
    meas_precompute_[i] = Eigen::Matrix<double,Eigen::Dynamic,1>(num_inliers);
    alpha_precompute_[i] = Eigen::Matrix<double,Eigen::Dynamic,1>(num_inliers);
    malpha_precompute_[i] = Eigen::Matrix<double,Eigen::Dynamic,1>(num_inliers);
    int k = 0;
    for (int j = 0; j < const_frames[i].size(); ++j) {
      if ((*best_inliers[i])(j)) {
        inlier_frames[i].push_back(const_frames[i][j]);
        ransac_precompute_[i].row(k) = ransac_precompute_temp[i].row(j);
        meas_precompute_[i][k] = meas_precompute_temp[i][j];
        alpha_precompute_[i][k] = std::min(1.0, std::max(0.0, (const_frames[i][j].timestamp - trajectory_.back().begin_timestamp) / 
            (trajectory_.back().end_timestamp - trajectory_.back().begin_timestamp)));
        malpha_precompute_[i][k] = std::max(0.0, 1.0 - alpha_precompute_[i][k]);
        k++;
      }
    }
  }
  LOG(INFO) << "before ransac: " << const_frames[0].size() << std::endl;
  LOG(INFO) << "after ransac: " << inlier_frames[0].size() << std::endl;
  return inlier_frames;
}

void DopplerFilter::registerFrame(const std::vector<Pointcloud> &const_frames, const std::vector<Eigen::MatrixXd> &gyro) {
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
  for (int i = 0; i < options_.num_sensors; ++i) {
    if (!sensor_active_[i])
      continue; // no measurements
    Eigen::Matrix<double, Eigen::Dynamic, 12> G(const_frames[i].size(), 12); // N x 12
    G.leftCols<6>() = ransac_precompute_[i].array().colwise() * malpha_precompute_[i].array();
    G.rightCols<6>() = ransac_precompute_[i].array().colwise() * alpha_precompute_[i].array();

    lhs += G.transpose() * G / (0.2*0.2);   // TODO: add variance as parameter
    rhs += G.transpose() * meas_precompute_[i] / (0.2*0.2);
  } // end for i

  // TODO: Is calling inverse on 6x6 slower than using a linear solver?
  // still need to check. This function takes 0.54ms on average atm though

  // marginalize
  Eigen::Matrix<double, 6, 6> temp = lhs.bottomLeftCorner<6,6>()*lhs.topLeftCorner<6,6>().inverse();
  Eigen::Matrix<double, 6, 6> lhs_new = lhs.bottomRightCorner<6,6>() - temp*lhs.topRightCorner<6,6>();
  Eigen::Matrix<double, 6, 1> rhs_new = rhs.tail<6>() - temp*rhs.head<6>();

  // solve
  trajectory_.back().varpi = lhs_new.inverse() * rhs_new;
  last_lhs_ = lhs_new;
  last_rhs_ = rhs_new;

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
