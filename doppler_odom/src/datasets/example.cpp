#include "doppler_odom/datasets/example.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>

namespace doppler_odom {

namespace {

Pointcloud simPointCloud(const std::string &path, const double& start_time, 
    const double& end_time, const Eigen::Matrix<double,3,6>& Tad, const Eigen::Matrix<double, 6, 1>& const_gt_vel) {

  // for our example, we will simulate a pointcloud frame with constant velocity
  const double const_range = 50.0;
  double hfov = 100 * M_PI / 180.0;
  double hres = 0.2 * M_PI / 180.0;
  int num_cols = int(hfov / hres);
  double vfov = 30 * M_PI / 180.0;
  double vres = 0.5 * M_PI / 180.0;
  int num_rows = int(vfov / vres);
  Pointcloud frame;
  frame.reserve(num_cols * num_rows);
  for (int r = 0; r < num_rows; ++r) {
    double elevation = -0.5 * vfov + r * vres;
    double z = const_range * sin(elevation);
    double xy = const_range * cos(elevation);

    for (int c = 0; c < num_cols; ++c) {
      double azimuth = -0.5 * hfov + c * hres;
      double x = xy * cos(azimuth);
      double y = xy * sin(azimuth);

      Point3D new_point;
      new_point.pt << x, y, z;  // coordinates
      Eigen::Vector3d unit_dir = new_point.pt / sqrt(x*x + y*y + z*z);
      new_point.radial_velocity = unit_dir.dot(Tad * const_gt_vel);
      new_point.timestamp = start_time + double(c) / num_cols * (end_time - start_time);
      new_point.beam_id = r;
      new_point.range = const_range;
      new_point.intensity = 1.0;
    }
  }
  return frame;
}

}  // namespace

Sequence::Ptr ExampleDataset::next() {
  if (!hasNext()) return nullptr;
  ExampleDataset::Options options(options_);
  options.sequence = sequences_[next_sequence_++];
  return std::make_shared<ExampleSequence>(options);
}

ExampleSequence::ExampleSequence(const ExampleDataset::Options& options) : options_(options) {

  /* initialize frame counting variables here */
  last_frame_ = std::min(1000, options_.last_frame);  // hard-coded 1000 for our simulation
  curr_frame_ = std::max((int)0, options_.init_frame);
  init_frame_ = std::max((int)0, options_.init_frame);

  /* set sensor calibration parameters here */
  /* must set extrinsics for FMCW lidars and inverse covariance for gyros */
  calib_ = std::make_shared<SensorCalib>();

  // extrinsic for FMCW lidar (values set for our simulation)
  Eigen::Matrix4d T_sv = Eigen::Matrix4d::Identity();
  T_sv(0, 3) = -1.0;
  T_sv(2, 3) = -1.0;

  // note: these members are vectors to handle multiple sensors (order corresponds to sensor_id field in point data)
  calib_->T_sv.push_back(T_sv); 
  calib_->adT_sv_top3rows.push_back(lgmath::se3::tranAd(calib_->T_sv[0]).topRows<3>()); // top 3 rows of adjoint of T_sv

  // inverse covariance for gyros
  // note: these members ar evectors to handle multiple gyros (order corresponds to output vector of nextGyro() function)
  Eigen::Matrix3d Rinv = Eigen::Matrix3d::Identity() * 1.0e3;
  calib_->gyro_invcov.push_back(Rinv);

  // constant 6DOF body-velocity for our simulation
  const_gt_vel_ << -8.0, 0.0, 0.0, 0.0, 0.0, 0.3;
}

// load next lidar frame (also return start and end times of frame)
Pointcloud ExampleSequence::next(double& start_time, double& end_time) {
  if (!hasNext()) throw std::runtime_error("No more frames in sequence");

  /* add code here to load current lidar frame */

  // code below is specific to our simulation example
  start_time = 0.1 * curr_frame_++;
  end_time = start_time + 0.1;

  // load pointcloud
  auto frame = simPointCloud("dummy", start_time, end_time, calib_->adT_sv_top3rows[0], const_gt_vel_);
  LOG(INFO) << "# points: " << frame.size() << std::endl;

  return frame;
}

// load gyro data between start_time and end_time
std::vector<Eigen::MatrixXd> ExampleSequence::nextGyro(const double& start_time, const double& end_time) {
  std::vector<Eigen::MatrixXd> output;
  return output;
}

// dataset-specific data preprocessing (e.g., downsampling, Doppler bias calibration, etc.)
Pointcloud ExampleSequence::preprocessFrame(Pointcloud& frame, double start_time, double end_time) {
  Pointcloud keypoint_frame = frame;  // do nothing
  return keypoint_frame;
}

void ExampleSequence::save(const std::string& path, const Trajectory& trajectory, const std::vector<Eigen::Matrix4d> &poses) const {
  std::ofstream trajectory_file;
  const auto vfilename = path + "/" + options_.sequence + "_velocity.txt";
  trajectory_file.open(vfilename, std::ios::out);
  trajectory_file << std::fixed << std::setprecision(12) << trajectory[0].begin_timestamp << " " << 0.0 << " " << 0.0 << " " 
    << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
  for (int i = 0; i < trajectory.size(); ++i) {
    trajectory_file << std::fixed << std::setprecision(12) << trajectory[i].end_timestamp << " " << trajectory[i].varpi(0) 
      << " " << trajectory[i].varpi(1) << " " << trajectory[i].varpi(2)
      << " " << trajectory[i].varpi(3) << " " << trajectory[i].varpi(4) 
      << " " << trajectory[i].varpi(5) << std::endl;
  }

  std::ofstream pose_file;
  const auto pfilename = path + "/" + options_.sequence + "_poses.txt";
  pose_file.open(pfilename, std::ios::out);
  for (int i = 0; i < poses.size(); ++i) {
    pose_file << std::fixed << std::setprecision(12) 
             << poses[i](0,0) << " " << poses[i](0,1) << " " << poses[i](0,2) << " " << poses[i](0,3)
      << " " << poses[i](1,0) << " " << poses[i](1,1) << " " << poses[i](1,2) << " " << poses[i](1,3)
      << " " << poses[i](2,0) << " " << poses[i](2,1) << " " << poses[i](2,2) << " " << poses[i](2,3)
      << " " << poses[i](3,0) << " " << poses[i](3,1) << " " << poses[i](3,2) << " " << poses[i](3,3) << std::endl;
  }
}

}  // namespace doppler_odom