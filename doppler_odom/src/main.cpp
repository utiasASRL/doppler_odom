#include <filesystem>
namespace fs = std::filesystem;

#include <iostream>
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include "lgmath.hpp"

#include "doppler_odom/dataset.hpp"
#include "doppler_odom/odometry.hpp"
#include "doppler_odom/odometry/doppler_filter.hpp"
#include "doppler_odom/point.hpp"
#include "doppler_odom/utils/stopwatch.hpp"
#include "doppler_odom/calib/doppler_calib.hpp"

namespace doppler_odom {

// Parameters to run the odometry
struct OdomOptions {
  std::string dataset;
  Dataset::Options dataset_options;

  std::string odometry;
  Odometry::Options::Ptr odometry_options;

  DopplerCalib::Options doppler_options;  
};

doppler_odom::OdomOptions loadOptions(const YAML::Node& config) {
  doppler_odom::OdomOptions options;
  options.dataset = config["dataset"].as<std::string>();
  options.odometry = config["odometry"].as<std::string>();

  // dataset
  auto& dataset_options = options.dataset_options;
  dataset_options.all_sequences = config["dataset_options"]["all_sequences"].as<bool>();
  dataset_options.root_path = config["dataset_options"]["root_path"].as<std::string>();
  dataset_options.sequence = config["dataset_options"]["sequence"] .as<std::string>();
  dataset_options.init_frame = config["dataset_options"]["init_frame"].as<int>();
  dataset_options.last_frame = config["dataset_options"]["last_frame"].as<int>();
  std::vector<bool> active_sensors = config["dataset_options"]["active_sensors"].as<std::vector<bool>>();
  dataset_options.active_sensors = active_sensors;

  // calib
  auto& doppler_options = options.doppler_options;
  doppler_options.root_path = config["doppler_options"]["root_path"].as<std::string>();
  doppler_options.azimuth_res = config["doppler_options"]["azimuth_res"].as<double>();
  doppler_options.azimuth_start = config["doppler_options"]["azimuth_start"].as<double>();
  doppler_options.azimuth_end = config["doppler_options"]["azimuth_end"].as<double>();
  doppler_options.num_rows = config["doppler_options"]["num_rows"].as<int>();
  doppler_options.num_cols = config["doppler_options"]["num_cols"].as<int>();
  doppler_options.active_sensors = active_sensors;

  // odometry
  if (options.odometry == "doppler_filter")
    options.odometry_options = std::make_shared<DopplerFilter::Options>();
  else
    throw std::invalid_argument{"Unknown odometry type!"};
  auto& odometry_options = *options.odometry_options;

  odometry_options.debug_path = config["log_dir"].as<std::string>();

  int num_sensors = 0;
  for (const auto& flag : active_sensors)
    if (flag)
      ++num_sensors;
  odometry_options.num_sensors = num_sensors;
  odometry_options.ransac_max_iter = config["odometry_options"]["ransac_max_iter"].as<int>();
  odometry_options.ransac_thres = config["odometry_options"]["ransac_thres"].as<double>();
  odometry_options.integration_steps = config["odometry_options"]["integration_steps"].as<int>();
  odometry_options.zero_vel_tol = config["odometry_options"]["zero_vel_tol"].as<double>();

  odometry_options.min_dist = config["odometry_options"]["min_dist_lidar_center"].as<double>();
  odometry_options.max_dist = config["odometry_options"]["max_dist_lidar_center"].as<double>();

  auto temp = config["odometry_options"]["P0inv"].as<std::vector<double>>();
  odometry_options.P0inv.diagonal() = Eigen::Matrix<double,6,1>(temp.data());

  temp = config["odometry_options"]["Qkinv"].as<std::vector<double>>();
  odometry_options.Qkinv.diagonal() = Eigen::Matrix<double,6,1>(temp.data());

  temp = config["odometry_options"]["Qzinv"].as<std::vector<double>>();
  odometry_options.Qzinv.diagonal() = Eigen::Matrix<double,6,1>(temp.data());

  // algorithm-specific parameters
  if (options.odometry == "doppler_filter") {
    if (DopplerFilter::Options* dfilter_options = dynamic_cast<DopplerFilter::Options*>(options.odometry_options.get())) {
      // gyro bias (1 for each sensor)
      auto temp_vec = config["odometry_options"]["const_gyro_bias"].as<std::vector<std::vector<double>>>();
      dfilter_options->const_gyro_bias.clear();
      for (auto& bias: temp_vec)
        dfilter_options->const_gyro_bias.push_back(Eigen::Vector3d(bias.data()));
    }
  }

  return options;
}

}  // namespace doppler_odom

int main(int argc, char** argv) {
  using namespace doppler_odom;

  // load yaml config
  std::string config_path = argv[1];
  YAML::Node config = YAML::LoadFile(config_path);

  // setup logging
  FLAGS_log_dir = config["log_dir"].as<std::string>();
  FLAGS_alsologtostderr = 1;
  fs::create_directories(FLAGS_log_dir);
  google::InitGoogleLogging(argv[0]);
  LOG(WARNING) << "Logging to " << FLAGS_log_dir;

  // read parameters
  const auto options = loadOptions(config);

  // get dataset
  const auto dataset = Dataset::Get(options.dataset, options.dataset_options);

  // doppler calibration
  const auto doppler_calib = std::make_shared<DopplerCalib>(options.doppler_options);

  // loop through sequences
  while (auto seq = dataset->next()) {
    LOG(WARNING) << "Running odometry on sequence: " << seq->name() << std::endl;

    // timers
    std::vector<std::pair<std::string, std::unique_ptr<Stopwatch<>>>> timer;
    timer.emplace_back("loading ..................... ", std::make_unique<Stopwatch<>>(false));
    timer.emplace_back("preprocess ..................... ", std::make_unique<Stopwatch<>>(false));
    timer.emplace_back("loading gyro ..................... ", std::make_unique<Stopwatch<>>(false));
    timer.emplace_back("ransac ................ ", std::make_unique<Stopwatch<>>(false));
    timer.emplace_back("solve ............... ", std::make_unique<Stopwatch<>>(false));
    timer.emplace_back("integration ............... ", std::make_unique<Stopwatch<>>(false));

    // get odometry
    auto odometry = Odometry::Get(options.odometry, *options.odometry_options);
    odometry->setDopplerCalib(doppler_calib);

    bool odometry_success = true;
    while (seq->hasNext()) {
      LOG(INFO) << "Processing frame " << seq->currFrame() << std::endl;

      // load next lidar frame (~13ms, not counted towards time in paper)
      timer[0].second->start();
      double start_time;
      double end_time;
      auto frames = seq->next(start_time, end_time);
      timer[0].second->stop();

      // preprocessing step (downsample + regression, ~3.9ms)
      timer[1].second->start();
      const auto preprocessed_frames = odometry->preprocessFrame(frames, start_time, end_time);
      timer[1].second->stop();

      // load gyro measurements that overlap with latest lidar frame (~0.1ms, not counted towards time in paper)
      timer[2].second->start();
      auto frame_times = odometry->getLatestFrameTimes();
      auto gyro = seq->next_gyro(frame_times[0], frame_times[1]);
      timer[2].second->stop();

      // ransac (~1.1ms)
      timer[3].second->start();
      const auto ransac_frames = odometry->ransacFrame(preprocessed_frames);
      timer[3].second->stop();

      // estimate latest velocity (~0.5ms)
      timer[4].second->start();
      odometry->registerFrame(ransac_frames, gyro);
      timer[4].second->stop();

      // integrate for latest pose (~0.01ms)
      timer[5].second->start();
      const auto pose = odometry->integrateForPose();
      timer[5].second->stop();
    }

    // dump timing information
    for (size_t i = 0; i < timer.size(); i++) {
      LOG(WARNING) << "Average " << timer[i].first << (timer[i].second->count() / (double)seq->numFrames()) << " ms"
                   << std::endl;
    }

    // save
    seq->save(config["log_dir"].as<std::string>(), odometry->trajectory(), odometry->poses());

  }

  return 0;
}