#include "doppler_odom/datasets/boreas_aeva.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>

#include "doppler_odom/datasets/utils.hpp"

namespace doppler_odom {

namespace {

inline Eigen::Matrix3d roll(const double &r) {
  Eigen::Matrix3d res;
  res << 1., 0., 0., 0., std::cos(r), std::sin(r), 0., -std::sin(r), std::cos(r);
  return res;
}

inline Eigen::Matrix3d pitch(const double &p) {
  Eigen::Matrix3d res;
  res << std::cos(p), 0., -std::sin(p), 0., 1., 0., std::sin(p), 0., std::cos(p);
  return res;
}

inline Eigen::Matrix3d yaw(const double &y) {
  Eigen::Matrix3d res;
  res << std::cos(y), std::sin(y), 0., -std::sin(y), std::cos(y), 0., 0., 0., 1.;
  return res;
}

inline Eigen::Matrix3d rpy2rot(const double &r, const double &p, const double &y) {
  return roll(r) * pitch(p) * yaw(y);
}

ArrayPoses loadPoses(const std::string &file_path) {
  ArrayPoses poses;
  std::ifstream pose_file(file_path);
  if (pose_file.is_open()) {
    std::string line;
    std::getline(pose_file, line);  // header
    for (; std::getline(pose_file, line);) {
      if (line.empty()) continue;
      std::stringstream ss(line);

      int64_t timestamp = 0;
      Eigen::Matrix4d T_ms = Eigen::Matrix4d::Identity();
      double r = 0, p = 0, y = 0;

      for (int i = 0; i < 10; ++i) {
        std::string value;
        std::getline(ss, value, ',');

        if (i == 0)
          timestamp = std::stol(value);
        else if (i == 1)
          T_ms(0, 3) = std::stod(value);
        else if (i == 2)
          T_ms(1, 3) = std::stod(value);
        else if (i == 3)
          T_ms(2, 3) = std::stod(value);
        else if (i == 7)
          r = std::stod(value);
        else if (i == 8)
          p = std::stod(value);
        else if (i == 9)
          y = std::stod(value);
      }
      T_ms.block<3, 3>(0, 0) = rpy2rot(r, p, y).transpose();

      (void)timestamp;
      poses.push_back(T_ms);
    }
  } else {
    throw std::runtime_error{"unable to open file: " + file_path};
  }
  return poses;
}

Pointcloud readPointCloud(const std::string &path, const double &time_delta_sec, double& start_time, double& end_time) {
  Pointcloud frame;

  // read bin file
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4;
  // unsigned fields = 8;  // x, y, z, i, r, t, b, s (note: s is sensor id)
  unsigned fields = 7;  // x, y, z, i, r, t, b
  unsigned point_step = float_offset * fields;
  unsigned numPointsIn = std::floor(buffer.size() / point_step);

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float { return *((float *)(byteArray + index)); };

  double frame_last_timestamp = -1000000.0;
  double frame_first_timestamp = 1000000.0;
  frame.reserve(numPointsIn); 
  for (unsigned i(0); i < numPointsIn; i++) {
    Point3D new_point;

    int bufpos = i * point_step;
    int offset = 0;
    new_point.pt[0] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.pt[1] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.pt[2] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    ++offset;
    // intensity skipped
    ++offset;
    new_point.radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.timestamp = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    ++offset;
    new_point.beam_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    if (new_point.timestamp < frame_first_timestamp) {
      frame_first_timestamp = new_point.timestamp;
    }

    if (new_point.timestamp > frame_last_timestamp) {
      frame_last_timestamp = new_point.timestamp;
    }

    frame.push_back(new_point);
  }
  frame.shrink_to_fit();
  for (int i(0); i < (int)frame.size(); i++) {
    frame[i].timestamp += time_delta_sec;
  }

  start_time = frame_first_timestamp + time_delta_sec;
  end_time = frame_last_timestamp + time_delta_sec;
  return frame;
}

}  // namespace

bool filecomp (std::string file1, std::string file2) { 
  long long i = std::stoll(file1.substr(0, file1.find(".")));
  long long j = std::stoll(file2.substr(0, file2.find(".")));
  return (i<j); 
}

BoreasAevaSequence::BoreasAevaSequence(const Options &options) : Sequence(options) {
  dir_path_ = options_.root_path + "/" + options_.sequence + "/aeva/";
  auto dir_iter = std::filesystem::directory_iterator(dir_path_);
  last_frame_ = std::count_if(begin(dir_iter), end(dir_iter), [this](auto &entry) {
    if (entry.is_regular_file()) filenames_.emplace_back(entry.path().filename().string());
    return entry.is_regular_file();
  });
  last_frame_ = std::min(last_frame_, options_.last_frame);
  curr_frame_ = std::max((int)0, options_.init_frame);
  init_frame_ = std::max((int)0, options_.init_frame);
  std::sort(filenames_.begin(), filenames_.end(), filecomp);  // custom comparison

  initial_timestamp_micro_ = std::stoll(filenames_[0].substr(0, filenames_[0].find(".")));

  // read gyro measurements
  // TODO: handle multiple sensors for gyro
  gyro_data_.clear();
  std::string gyro_path = options_.root_path + "/" + options_.sequence + "/applanix/" + "aeva_imu.csv";
  gyro_data_.push_back(readBoreasGyroToEigenXd(gyro_path, initial_timestamp_micro_));
  LOG(INFO) << "Loaded gyro data " << ". Matrix " 
      << gyro_data_.back().rows() << " x " << gyro_data_.back().cols() << std::endl;
}

std::vector<Pointcloud> BoreasAevaSequence::next(double& start_time, double& end_time) {
  if (!hasNext()) throw std::runtime_error("No more frames in sequence");
  int curr_frame = curr_frame_++;
  auto filename = filenames_.at(curr_frame);
  int64_t time_delta_micro = std::stoll(filename.substr(0, filename.find("."))) - initial_timestamp_micro_;
  // int64_t time_delta_micro = std::stoll(filename.substr(0, filename.find(".")));
  double time_delta_sec = static_cast<double>(time_delta_micro) / 1e6;

  // load point cloud
  auto frame = readPointCloud(dir_path_ + "/" + filename, time_delta_sec, start_time, end_time);

  // TODO: logic for reading from more than 1 sensor
  std::vector<Pointcloud> frames;
  frames.push_back(frame);

  LOG(INFO) << "# points: " << frames.size() << ", " << frames[0].size() << std::endl;

  return frames;
}

std::vector<Eigen::MatrixXd> BoreasAevaSequence::next_gyro(const double& start_time, const double& end_time) {
  // double dt = 0.0;
  double dt = 0.1;
  std::vector<Eigen::MatrixXd> output;
  for (int sensorid = 0; sensorid < gyro_data_.size(); ++sensorid) {
    std::vector<int> inds; inds.clear();
    for (int r = 0; r < gyro_data_[sensorid].rows(); ++r) {
      double meas_time = gyro_data_[sensorid](r, 0) - dt;
      if (meas_time >= start_time && meas_time < end_time)
        inds.push_back(r);
    } // end for r

    if (inds.size() == 0) {
      // no measurements
      output.push_back(Eigen::Matrix<double, 1, 1>());  // 1x1 zero matrix
      LOG(INFO) << "grabbing gyro " << sensorid << ", no gyro data" << std::endl;
      continue;
    }

    Eigen::MatrixXd temp_gyro(inds.size(), 4);
    for (int r = 0; r < inds.size(); ++r) {
      temp_gyro(r, 0) = gyro_data_[sensorid](inds[r], 0) - dt; // timestamp
      temp_gyro.row(r).rightCols<3>() = gyro_data_[sensorid].row(inds[r]).rightCols<3>();
    }
    output.push_back(temp_gyro);
    LOG(INFO) << "grabbing gyro " << sensorid << ", " << output.back().rows() << " x " << output.back().cols() 
      << ". Start time: " << output.back()(0, 0) << ", " << "end time: " << output.back()(inds.size()-1, 0) << std::endl;
  } // end for sensorid
  return output;
}

void BoreasAevaSequence::save(const std::string& path, const Trajectory& trajectory, const std::vector<Eigen::Matrix4d> &poses) const {
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