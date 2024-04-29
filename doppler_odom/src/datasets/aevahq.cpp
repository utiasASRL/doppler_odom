#include "doppler_odom/datasets/aevahq.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>

#include "doppler_odom/datasets/utils.hpp"

namespace doppler_odom {

namespace {

Pointcloud readPointCloud(const std::string& path, double time_delta_sec, int sensor_id, double& start_time, double& end_time) {
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
    new_point.timestamp = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.beam_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.line_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    new_point.face_id = (int)getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    // for simplicity, we'll just reject points that are not within the start and end times
    if (new_point.timestamp < frame_first_timestamp || new_point.timestamp > frame_last_timestamp)
      continue;

    frame.push_back(new_point);
  }
  frame.shrink_to_fit();
  for (int i(0); i < (int)frame.size(); i++) {
    frame[i].timestamp += time_delta_sec;
  }

  start_time = time_delta_sec; // frame_first_timestamp + time_delta_sec;
  end_time = frame_last_timestamp + time_delta_sec;
  return frame;
}

}  // namespace

AevaHQSequence::AevaHQSequence(const Options &options) : Sequence(options) {
  // we will always index in this order:
  // 0: front-facing sensor, 1: left-facing sensor, 2: right-facing sensor, 3: back-facing sensor
  dir_path_[0] = options_.root_path + "/" + options_.sequence + "/front_4320/";
  dir_path_[1] = options_.root_path + "/" + options_.sequence + "/left_4386/";
  dir_path_[2] = options_.root_path + "/" + options_.sequence + "/right_4347/";
  dir_path_[3] = options_.root_path + "/" + options_.sequence + "/back_4363/";

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

  // initialize curr_frame and make sure lengths are the same 
  int len = last_frame_[0] - init_frame_[0];
  curr_frame_[0] = init_frame_[0] + std::max((int)0, options_.init_frame);;
  for (int i = 1; i < 4; ++i) {
    curr_frame_[i] = init_frame_[i] + std::max((int)0, options_.init_frame);;
    if (len != last_frame_[i] - init_frame_[i])
      throw std::runtime_error("Sensor " + std::to_string(i) 
        + " has " + std::to_string(last_frame_[i] - init_frame_[i]) 
        + " frames, instead of " + std::to_string(len) + " (Sensor 0)");
  }

  // set initial time to keep floats small
  initial_timestamp_micro_ = std::stoll(filenames_[0][init_frame_[0]].substr(0, filenames_[0][init_frame_[0]].find(".")));
}

Pointcloud AevaHQSequence::next(double& start_time, double& end_time) {
  if (!hasNext()) throw std::runtime_error("No more frames in sequence");

  // load active sensors
  Pointcloud output_frame;
  for (int sensor_id = 0; sensor_id < 4; ++sensor_id) {
    // skip if inactive
    if (options_.active_sensors[sensor_id] == false)
      continue;
    
    // frame_id, filename, and start time
    int curr_frame = curr_frame_[sensor_id]++;  // grab frame id and increment after
    auto& filename = filenames_[sensor_id].at(curr_frame);
    int64_t time_delta_micro = std::stoll(filename.substr(0, filename.find("."))) - initial_timestamp_micro_;
    double time_delta_sec = static_cast<double>(time_delta_micro) / 1e6;

    // load and concatenate
    auto frame = readPointCloud(dir_path_[0] + "/" + filename, time_delta_sec, sensor_id, start_time, end_time);
    output_frame.insert(
      output_frame.end(),
      std::make_move_iterator(frame.begin()),
      std::make_move_iterator(frame.end())
    );
  }

  LOG(INFO) << "# points: " << output_frame.size() << std::endl;

  return output_frame;
}

std::vector<Eigen::MatrixXd> AevaHQSequence::next_gyro(const double& start_time, const double& end_time) {
  std::vector<Eigen::MatrixXd> output;
  return output;
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