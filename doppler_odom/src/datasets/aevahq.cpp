#include "doppler_odom/datasets/aevahq.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>

namespace doppler_odom {

namespace {

Pointcloud readPointCloud(const std::string& path, double time_delta_sec, int sensor_id, double start_time, double end_time) {
  Pointcloud frame;

  // read bin file
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4;
  unsigned fields = 11;  // x, y, z, radial velocity, intensity, signal quality, reflectivity, time, beam_id, line_id, face_id
  unsigned point_step = float_offset * fields;
  unsigned numPointsIn = std::floor(buffer.size() / point_step);

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float { return *((float *)(byteArray + index)); };

  double frame_last_timestamp = -1000000.0;
  double frame_first_timestamp = 1000000.0;
  frame.reserve(numPointsIn); 
  for (unsigned i(0); i < numPointsIn; i++) {
    Point3D new_point;

    // x y z
    int bufpos = i * point_step;
    int offset = 0;
    new_point.pt[0] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.pt[1] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.pt[2] = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    // others
    ++offset;
    new_point.radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.intensity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // signal quality skipped
    ++offset;
    // reflectivity skipped
    ++offset;
    new_point.timestamp = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset) * 1e-9 + time_delta_sec; // time is in nanoseconds
    ++offset;
    new_point.beam_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.line_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.face_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    new_point.sensor_id = sensor_id;

    // error checks
    if (new_point.line_id < 0 || new_point.line_id >= 64)
      continue;
    if (new_point.face_id < 0 || new_point.face_id > 5)
      continue;
    if (new_point.sensor_id < 0 || new_point.sensor_id > 4)
      continue;

    // include if within start and end time
    if (new_point.timestamp >= start_time && new_point.timestamp <= end_time)
      frame.push_back(new_point);
  }

  frame.shrink_to_fit();
  return frame;
}

}  // namespace

Sequence::Ptr AevaHQDataset::next() {
  if (!hasNext()) return nullptr;
  AevaHQDataset::Options options(options_);
  options.sequence = sequences_[next_sequence_++];
  return std::make_shared<AevaHQSequence>(options);
}

AevaHQSequence::AevaHQSequence(const AevaHQDataset::Options& options) : options_(options) {
  // we will always index in this order:
  // 0: front-facing sensor, 1: left-facing sensor, 2: right-facing sensor, 3: back-facing sensor
  std::string lnames[4];  // lidar names
  lnames[0] = "front_4320";
  lnames[1] = "left_4386";
  lnames[2] = "right_4347";
  lnames[3] = "back_4363";
  for (size_t i = 0; i < 4; ++i)
    dir_path_[i] = options_.root_path + "/" + options_.sequence + "/" + lnames[i] + "/";

  // get filenames for each sensor (4 sensors total)
  for (int i = 0; i < 4; ++i) {
    filenames_.push_back(std::vector<std::string>());
    auto dir_iter = std::filesystem::directory_iterator(dir_path_[i]);
    last_frame_[i] = std::count_if(begin(dir_iter), end(dir_iter), [&](auto &entry) {
      if (entry.is_regular_file()) filenames_[i].emplace_back(entry.path().filename().string());
      return entry.is_regular_file();
    });
    std::sort(filenames_[i].begin(), filenames_[i].end(), filecomp);  // custom comparison
  }

  // the sensor frames are synchronized, but may have an extra frame or two at the start which we need to ignore
  // set init_frame_[x] for each sensor so they all start at the same time
  while ([this]() -> bool {
      // compare times and find min
      int64_t time_micro[4];
      bool eq_flag = true;  // results in true if all frames are equal to each other
      int64_t min_time = 0; int min_id = 0;
      int64_t tol = 0.015 * 1e6;  // 0.015 seconds to microseconds
      for (int i = 0; i < 4; ++i) {
        std::string& filename = filenames_[i][init_frame_[i]];
        time_micro[i] = std::stoll(filename.substr(0, filename.find("."))); // string to int
        if (i == 0) {
          min_time = time_micro[0];
          min_id = 0;
          continue;
        }

        // compare i to 0 and update min
        eq_flag = eq_flag && (std::abs(time_micro[i] - time_micro[0]) < tol);
        if (time_micro[i] < min_time) {
          min_time = time_micro[i];
          min_id = i;
        }
      }

      if (eq_flag)
        return false; // exit while loop
      else {
        ++init_frame_[min_id];  // increment smallest frame by 1
        return true;  // continue while loop
      }
    }()
  );

  // initialize curr_frame_ for each sensor and make sure lengths are the same 
  // if options_.init_frame != 0, we need to offset curr_frame_ for each sensor.
  int len = last_frame_[0] - init_frame_[0];
  curr_frame_[0] = init_frame_[0] + std::max((int)0, options_.init_frame);
  for (int i = 1; i < 4; ++i) {
    curr_frame_[i] = init_frame_[i] + std::max((int)0, options_.init_frame);
    if (len != last_frame_[i] - init_frame_[i])
      throw std::runtime_error("Sensor " + std::to_string(i) 
        + " has " + std::to_string(last_frame_[i] - init_frame_[i]) 
        + " frames, instead of " + std::to_string(len) + " (Sensor 0)");
  }

  // set initial time to keep floats small
  initial_timestamp_micro_ = std::stoll(filenames_[0][init_frame_[0]].substr(0, filenames_[0][init_frame_[0]].find(".")));

  // gyro
  gyro_data_.clear();
  const_gyro_bias_.clear();
  for (size_t i = 0; i < 4; ++i) {
    std::string gyro_path = options_.root_path + "/" + options_.sequence + "/" + lnames[i] + "_imu.csv";
    gyro_data_.push_back(readGyroToEigenXd(gyro_path, initial_timestamp_micro_, "aevahq"));
    gyro_data_.back().rightCols<3>() *= -1.0; // flip reference frame
    LOG(INFO) << "Loaded gyro data " << ". Matrix " 
        << gyro_data_.back().rows() << " x " << gyro_data_.back().cols() << std::endl;

    // calibrate gyro bias using first N measurements while stationary
    const_gyro_bias_.push_back(gyro_data_.back().topRightCorner(options_.nframes_gbias_calib, 3).colwise().mean().transpose());
  }

  // other calib
  calib_ = std::make_shared<SensorCalib>();

  // initialize gyro
  calib_->gyro_invcov.clear();
  for (const auto& gyro_ivar: options_.gyro_ivar) {
    Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();
    temp.diagonal() = Eigen::Vector3d(gyro_ivar.data());
    calib_->gyro_invcov.push_back(temp);
  }

  // init sensor extrinsics
  calib_->T_sv.clear();
  calib_->adT_sv_top3rows.clear();
  for (const auto& xi_sv: options_.xi_sv) {
    const auto T_sv = lgmath::se3::Transformation(Eigen::Matrix<double, 6, 1>(xi_sv.data()));
    calib_->T_sv.push_back(T_sv.matrix());
    calib_->adT_sv_top3rows.push_back(lgmath::se3::tranAd(T_sv.matrix()).topRows<3>());
  }

  // Doppler image space calibration
  doppler_image_calib_ = std::make_shared<DopplerImageCalib>(options_.dcalib_options);
}

// load next lidar frame from (possibly) multiple sensors (also return start and end times of frame)
Pointcloud AevaHQSequence::next(double& start_time, double& end_time) {
  if (!hasNext()) throw std::runtime_error("No more frames in sequence");

  // Use sensor 0 for start/end time
  // Note: we peak into future data for the end timestamp for evaluation convenience. An online implementation
  // would need different logic, i.e., use the last timestamp of the pointcloud
  int tsensorid = 0;
  int tframeid = curr_frame_[tsensorid];
  auto& start_name = filenames_[tsensorid].at(tframeid);  // time in microseconds as string
  start_time = static_cast<double>(std::stoll(start_name.substr(0, start_name.find("."))) - initial_timestamp_micro_) / 1e6;  // seconds
  if (tframeid + 1 < filenames_[tsensorid].size()) {
    auto& end_name = filenames_[tsensorid].at(tframeid + 1);
    end_time = static_cast<double>(std::stoll(end_name.substr(0, end_name.find("."))) - initial_timestamp_micro_) / 1e6;
  }
  else {
    end_time = start_time + 0.1;  // this will occur at the last frame, but we know it will approximately be 0.1 seconds
  }

  // TODO: apply azimuth and range scale calibration

  // load active sensors
  Pointcloud output_frame;
  for (int sensor_id = 0; sensor_id < 4; ++sensor_id) {
    
    // frame_id, filename, and start time
    int curr_frame = curr_frame_[sensor_id]++;  // grab frame id and increment after
    auto& filename = filenames_[sensor_id].at(curr_frame);
    int64_t time_delta_micro = std::stoll(filename.substr(0, filename.find("."))) - initial_timestamp_micro_;
    double time_delta_sec = static_cast<double>(time_delta_micro) / 1e6;

    // skip if inactive
    if (options_.active_lidars[sensor_id] == false)
      continue;

    // load and concatenate
    auto frame = readPointCloud(dir_path_[sensor_id] + "/" + filename, time_delta_sec, sensor_id, start_time, end_time);
    output_frame.insert(
      output_frame.end(),
      std::make_move_iterator(frame.begin()),
      std::make_move_iterator(frame.end())
    );
  }

  LOG(INFO) << "# points: " << output_frame.size() << std::endl;

  return output_frame;
}

// load gyro data between start_time and end_time
std::vector<Eigen::MatrixXd> AevaHQSequence::nextGyro(const double& start_time, const double& end_time) {
  // // TODO
  // std::vector<Eigen::MatrixXd> output;
  // output.resize(4, Eigen::MatrixXd(0, 0));
  // return output;
  // double dt = 0.0;

  std::vector<Eigen::MatrixXd> output;
  for (int sensorid = 0; sensorid < gyro_data_.size(); ++sensorid) {
    if (options_.active_gyros[sensorid] != true) {    // inactive gyro
      output.push_back(Eigen::MatrixXd(0, 0));  // empty matrix
      continue;
    }

    // find indices for data between start and end times
    std::vector<int> inds; inds.clear();
    for (int r = 0; r < gyro_data_[sensorid].rows(); ++r) {
      double meas_time = gyro_data_[sensorid](r, 0);
      if (meas_time >= start_time && meas_time < end_time)
        inds.push_back(r);
    } // end for r

    if (inds.size() == 0) {   // no measurements
      output.push_back(Eigen::MatrixXd(0, 0));  // empty matrix
      LOG(INFO) << "grabbing gyro " << sensorid << ", no gyro data" << std::endl;
      continue;
    }

    // output
    Eigen::MatrixXd temp_gyro(inds.size(), 4);
    for (int r = 0; r < inds.size(); ++r) {
      temp_gyro(r, 0) = gyro_data_[sensorid](inds[r], 0); // timestamp
      temp_gyro.row(r).rightCols<3>() = gyro_data_[sensorid].row(inds[r]).rightCols<3>() 
          - const_gyro_bias_[sensorid].transpose(); // measurement w/ bias comp.
    }
    output.push_back(temp_gyro);
    LOG(INFO) << "grabbing gyro " << sensorid << ", " << output.back().rows() << " x " << output.back().cols() 
      << ". Start time: " << output.back()(0, 0) << ", " << "end time: " << output.back()(inds.size()-1, 0) << std::endl;
  } // end for sensorid
  if (output.size() != 4)
    throw std::runtime_error("[AevaHQSequence::nextGyro] invalid number of gyro sensors"); 
    
  return output;
}

// dataset-specific data preprocessing (e.g., downsampling, Doppler bias calibration, etc.)
Pointcloud AevaHQSequence::preprocessFrame(Pointcloud& frame, double start_time, double end_time) {
  // image space calibration
  Pointcloud keypoint_frame = doppler_image_calib_->calib_frame(frame);  // downsamples into image and runs regression
  return keypoint_frame;
}

void AevaHQSequence::save(const std::string& path, const Trajectory& trajectory, const std::vector<Eigen::Matrix4d> &poses) const {
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