#pragma once

#include "doppler_odom/trajectory.hpp"
#include "doppler_odom/calib/doppler_calib.hpp"

#include <random>

namespace doppler_odom {

class Odometry {
 public:
  using Ptr = std::shared_ptr<Odometry>;
  using ConstPtr = std::shared_ptr<const Odometry>;

  struct Options {
    using Ptr = std::shared_ptr<Options>;
    using ConstPtr = std::shared_ptr<const Options>;

    virtual ~Options() = default;

    // sensor options
    int num_sensors = 1;
    double min_dist = 5.0;
    double max_dist = 150.0;

    // ransac options
    int ransac_max_iter = 20;
    double ransac_thres = 0.3;
    double ransac_min_range = 20.0;

    // integration
    int integration_steps = 100;
    double zero_vel_tol = 0.03;

    // inverse covariances
    Eigen::Matrix<double, 6, 6> Qkinv = Eigen::Matrix<double, 6, 6>::Identity(); 
    Eigen::Matrix<double, 6, 6> P0inv = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Qzinv = Eigen::Matrix<double, 6, 6>::Identity();

    //
    bool debug_print = false;  // Whether to output debug information to std::cout
    std::string debug_path = "/tmp/";
  };

  static Odometry::Ptr Get(const std::string &odometry, const Options &options) {
    return name2Ctor().at(odometry)(options);
  }

  Odometry(const Options &options) : options_(options) {}
  virtual ~Odometry() = default;

  // register new frame for odometry
  virtual void registerFrame(const std::vector<Pointcloud> &frame, const std::vector<Eigen::MatrixXd> &gyro) = 0;
  virtual std::vector<Pointcloud> preprocessFrame(std::vector<Pointcloud> &frame, const double& start_time, const double& end_time) = 0;
  virtual std::vector<Pointcloud> ransacFrame(const std::vector<Pointcloud> &frame) = 0;
  virtual Eigen::Matrix4d integrateForPose() = 0;
  virtual std::vector<double> getLatestFrameTimes() = 0;

  void setDopplerCalib(const DopplerCalib::ConstPtr& calib) {
    calib_ = calib;
  };

  Trajectory trajectory() {
    return trajectory_;
  };

  std::vector<Eigen::Matrix4d> poses() {
    return poses_;
  };

 protected:
  // precomputed measurement model (to avoid repeated calculations in RANSAC and main solve)
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,6>> ransac_precompute_;
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>> meas_precompute_;
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>> alpha_precompute_;
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>> malpha_precompute_;
  std::vector<bool> sensor_active_;

  Eigen::Matrix<double, 6, 6> last_lhs_;
  Eigen::Matrix<double, 6, 1> last_rhs_;

  long int seed_ = 0;
  std::mt19937_64 random_engine_;

  // extrinsic
  std::vector<Eigen::Matrix4d> T_sv_;
  std::vector<Eigen::Matrix<double,3,6>> adT_sv_top3rows_;

  // gyro inverse covariance
  std::vector<Eigen::Matrix3d> gyro_invcov_;

  Trajectory trajectory_;
  std::vector<Eigen::Matrix4d> poses_;

  DopplerCalib::ConstPtr calib_;

 private:
  const Options options_;

  using CtorFunc = std::function<Ptr(const Options &)>;
  using Name2Ctor = std::unordered_map<std::string, CtorFunc>;
  static Name2Ctor &name2Ctor() {
    static Name2Ctor name2ctor;
    return name2ctor;
  }

  template <typename T>
  friend class OdometryRegister;
};

template <typename T>
struct OdometryRegister {
  OdometryRegister() {
    bool success = Odometry::name2Ctor()
                       .try_emplace(T::odometry_name_, Odometry::CtorFunc([](const Odometry::Options &options) {
                                      return std::make_shared<T>(dynamic_cast<const typename T::Options &>(options));
                                    }))
                       .second;
    if (!success) throw std::runtime_error{"OdometryRegister failed - duplicated name"};
  }
};

#define DOPPLER_ODOM_REGISTER_ODOMETRY(NAME, TYPE)       \
 public:                                              \
  inline static constexpr auto odometry_name_ = NAME; \
                                                      \
 private:                                             \
  inline static doppler_odom::OdometryRegister<TYPE> odometry_reg_;

}  // namespace doppler_odom

///
#include "doppler_odom/odometry/doppler_filter.hpp"