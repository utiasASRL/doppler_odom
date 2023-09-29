#pragma once

#include <memory>
#include <string>
#include <vector>

#include "doppler_odom/point.hpp"
#include "doppler_odom/trajectory.hpp"

namespace doppler_odom {

using ArrayMatrix4d = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
using ArrayPoses = ArrayMatrix4d;

class Sequence {
 public:
  using Ptr = std::shared_ptr<Sequence>;
  using ConstPtr = std::shared_ptr<const Sequence>;

  struct Options {
    std::string root_path;
    std::string sequence;
    int init_frame = 0;
    int last_frame = std::numeric_limits<int>::max();  // exclusive bound
    std::vector<bool> active_sensors;
  };

  Sequence(const Options &options) : options_(options) {}
  virtual ~Sequence() = default;

  std::string name() const { return options_.sequence; }
  virtual int currFrame() const = 0;
  virtual int numFrames() const = 0;
  virtual void setInitFrame(int /* frame_index */) {
    throw std::runtime_error("set random initial frame not supported");
  };
  virtual bool hasNext() const = 0;
  virtual std::vector<Pointcloud> next(double& start_time, double& end_time) = 0;
  virtual bool withRandomAccess() const { return false; }
  virtual std::vector<Point3D> frame(size_t /* index */) const {
    throw std::runtime_error("random access not supported");
  }

  virtual bool hasGroundTruth() const { return false; }
  virtual void save(const std::string& path, const Trajectory& trajectory, const std::vector<Eigen::Matrix4d>& poses) const = 0;

  virtual std::vector<Eigen::MatrixXd> next_gyro(const double& start_time, const double& end_time) = 0;

 protected:
  const Options options_;
};

class Dataset {
 public:
  using Ptr = std::shared_ptr<Dataset>;
  using ConstPtr = std::shared_ptr<const Dataset>;

  struct Options : public Sequence::Options {
    bool all_sequences = false;
  };

  static Dataset::Ptr Get(const std::string &dataset, const Options &options) {
    return name2Ctor().at(dataset)(options);
  }

  Dataset(const Options &options) : options_(options) {}
  virtual ~Dataset() = default;

  virtual bool hasNext() const = 0;
  virtual Sequence::Ptr next() = 0;

 protected:
  const Options options_;

 private:
  using CtorFunc = std::function<Ptr(const Options &)>;
  using Name2Ctor = std::unordered_map<std::string, CtorFunc>;
  static Name2Ctor &name2Ctor() {
    static Name2Ctor name2ctor;
    return name2ctor;
  }

  template <typename T>
  friend class DatasetRegister;
};

template <typename T>
struct DatasetRegister {
  DatasetRegister() {
    bool success = Dataset::name2Ctor()
                       .try_emplace(T::dataset_name_, Dataset::CtorFunc([](const Dataset::Options &options) {
                                      return std::make_shared<T>(options);
                                    }))
                       .second;
    if (!success) throw std::runtime_error{"DatasetRegister failed - duplicated name"};
  }
};

#define DOPPLER_ODOM_REGISTER_DATASET(NAME, TYPE)       \
 public:                                             \
  inline static constexpr auto dataset_name_ = NAME; \
                                                     \
 private:                                            \
  inline static doppler_odom::DatasetRegister<TYPE> dataset_reg_;

}  // namespace doppler_odom

///
#include "doppler_odom/datasets/boreas_aeva.hpp"