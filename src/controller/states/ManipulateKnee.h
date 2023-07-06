#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include "../../bonetag/BoneTagSerial.h"
#include <deque>

struct ReadCSV
{
  void clear();
  void load(const std::string & path);

  std::deque<Eigen::Vector3d> femurTranslationVector;
  std::deque<Eigen::Vector3d> femurRotationVector;
  std::deque<Eigen::Vector3d> tibiaTranslationVector;
  std::deque<Eigen::Vector3d> tibiaRotationVector;
};

struct PTransformInterpolator
{
  sva::PTransformd operator()(const sva::PTransformd & p1, const sva::PTransformd & p2, double t)
  {
    return sva::interpolate(p1, p2, t);
  }
};

struct Result
{
  // Robot effective motion
  Eigen::Vector3d femurRotation;
  Eigen::Vector3d tibiaRotation;
  Eigen::Vector3d femurTranslation;
  Eigen::Vector3d tibiaTranslation;

  // Sensor measurements
  io::BoneTagSerial::Data sensorData;

  std::string to_csv() const;
};

struct ResultHandler
{
  void write_csv(const std::string & path);

  void clear()
  {
    results_.clear();
  }

  void addResult(const Result & result)
  {
    results_.push_back(result);
  }

protected:
  std::vector<Result> results_;
};

struct ManipulateKnee : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void resetToZero();

  void play()
  {
    if(file_.tibiaRotationVector.empty())
    {
      mc_rtc::log::error("[{}] Cannot play, empty file", name());
      return;
    }
    resultPath_ = results_dir_ + "/" + trajectory_file_;
    play_ = true;
    measuredSamples_ = 0;
  }

  void stop()
  {
    play_ = false;
    saveResults();
    trajOffsets_.reset();
  }

  void saveResults()
  {
    results_.write_csv(resultPath_);
    results_.clear();
  }

  inline void forceNext() noexcept
  {
    mc_rtc::log::warning("[{}] Manually forcing next position/measurement", name());
    iter_ = 0;
    measuredSamples_ = 0;
    next_ = true;
  }

  void setRate(double rate, double timeStep)
  {
    iterRate_ = std::max(1u, static_cast<unsigned>(ceil(1 / (1 / rate * timeStep))));
  }

  double getRate(double timeStep) const noexcept
  {
    return iterRate_ * timeStep;
  }

  inline void updateTibiaOffset(const sva::PTransformd & offset)
  {
    tibiaOffsetInterpolationTime_ = 0;
    tibiaOffsetInterpolator_.values({{0., tibiaOffset_}, {offsetDuration_, offset}});
  }

  inline void updateFemurOffset(const sva::PTransformd & offset)
  {
    femurOffsetInterpolationTime_ = 0;
    femurOffsetInterpolator_.values({{0., femurOffset_}, {offsetDuration_, offset}});
  }

  /// Rotation axis for the knee joint
  void updateAxes(mc_control::fsm::Controller & ctl);

  /// Returns true when all measurements have been taken
  bool measure(mc_control::fsm::Controller & ctl);

protected:
  ReadCSV file_;
  std::string trajectory_dir_ = "";
  std::string trajectory_file_ = "";

  bool play_ = false;
  bool next_ = true;
  size_t iter_ = 0;
  size_t iterRate_ = 1;
  double translationTreshold_ = 0.1; ///< Convergence threshold on translation [mm]
  double rotationTreshold_ = 0.1; ///< Convergence threshold on rotation [deg]

  unsigned desiredSamples_ = 10;
  unsigned measuredSamples_ = 0;

  ResultHandler results_;
  std::string results_dir_ = "/tmp";
  std::string resultPath_ = "/tmp/BoneTagResults.csv";

  // Replay trajectory with offsets
  bool repeatWithOffset_ =
      false; /// When true replay the trajectory from the start with an added offset from trajOffsets_
  struct TrajectoryOffsets
  {
    TrajectoryOffsets()
    {
      reset();
    }

    void next()
    {
      tibiaOffsetTranslation_ = tibiaOffsetsTranslation_[offsetIndex_];
      femurOffsetTranslation_ = femurOffsetsTranslation_[offsetIndex_];
      offsetIndex_ = std::min(offsetIndex_ + 1, offsetsNum_);
    }

    bool done() const noexcept
    {
      return offsetIndex_ == offsetsNum_;
    }

    void reset()
    {
      offsetIndex_ = 0;
      tibiaOffsetTranslation_ = Eigen::Vector3d::Zero();
      femurOffsetTranslation_ = Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d & tibiaOffsetTranslation() const noexcept
    {
      return tibiaOffsetTranslation_;
    }

    const Eigen::Vector3d & femurOffsetTranslation() const noexcept
    {
      return femurOffsetTranslation_;
    }

    unsigned current() const noexcept
    {
      return offsetIndex_;
    }

    unsigned size() const noexcept
    {
      return offsetsNum_;
    }

  protected:
    unsigned offsetIndex_ = 0;
    std::vector<Eigen::Vector3d> tibiaOffsetsTranslation_{{10., 0., 0.},  {0., 10., 0.}, {-10., 0., 0.},
                                                          {0., -10., 0.}, {0., 0., 10.}, {0., 0., -10.}};
    std::vector<Eigen::Vector3d> femurOffsetsTranslation_ = tibiaOffsetsTranslation_;
    unsigned offsetsNum_ = tibiaOffsetsTranslation_.size();
    Eigen::Vector3d tibiaOffsetTranslation_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d femurOffsetTranslation_ = Eigen::Vector3d::Zero();

  } trajOffsets_;

  sva::PTransformd X_0_femurAxis =
      sva::PTransformd::Identity(); ///< Axis around which the Femur frame rotates and translates
  sva::PTransformd X_0_tibiaAxis =
      sva::PTransformd::Identity(); ///< Axis around which the Tibia frame rotates and translates

  Eigen::Vector3d femurTranslation_ = Eigen::Vector3d::Zero(); ///< Desired joint translation in [mm]
  Eigen::Vector3d femurTranslationActual_ = Eigen::Vector3d::Zero(); ///< Current joint translation in [mm]
  Eigen::Vector3d minFemurTranslation_{-20, -20, -10}; ///< Min translation [mm]
  Eigen::Vector3d maxFemurTranslation_ = {20, 20, 10}; ///< Max translation [mm]

  Eigen::Vector3d tibiaTranslation_ = Eigen::Vector3d::Zero(); ///< Desired Joint translation in [mm]
  Eigen::Vector3d tibiaTranslationActual_ = Eigen::Vector3d::Zero(); ///< Current joint translation in [mm]
  sva::MotionVecd tibiaError_ = sva::MotionVecd::Zero();
  sva::MotionVecd femurError_ = sva::MotionVecd::Zero();
  Eigen::Vector3d minTibiaTranslation_{-20, -20, -10}; ///< Min translation [mm]
  Eigen::Vector3d maxTibiaTranslation_ = {20, 20, 10}; ///< Max translation [mm]

  Eigen::Vector3d tibiaRotation_ = Eigen::Vector3d::Zero(); ///< Desired joint rotation in [deg]
  Eigen::Vector3d tibiaRotationActual_ = Eigen::Vector3d::Zero(); ///< Current joint rotation in [deg]
  Eigen::Vector3d minTibiaRotation_{-20, -10, -10}; ///< Min Allowed Rotation [deg]
  Eigen::Vector3d maxTibiaRotation_ = {20, 10, 10}; ///< Max Allowed Rotation [deg]

  Eigen::Vector3d femurRotation_ = Eigen::Vector3d::Zero(); ///< Desired Joint rotation in [deg]
  Eigen::Vector3d femurRotationActual_ = Eigen::Vector3d::Zero(); ///< Current joint rotation in [deg]
  Eigen::Vector3d minFemurRotation_{-20, -10, -10}; ///< Min Allowed Rotation [deg]
  Eigen::Vector3d maxFemurRotation_ = {20, 10, 10}; ///< Max Allowed Rotation [deg]

  sva::PTransformd tibiaOffset_ = sva::PTransformd::Identity(); // Tibia offset [m]
  sva::PTransformd femurOffset_ = sva::PTransformd::Identity(); // Femur offset [m]
  sva::PTransformd tibiaOffsetInitial_ = sva::PTransformd::Identity(); // Initial Tibia offset (used when resetting) [m]
  sva::PTransformd femurOffsetInitial_ = sva::PTransformd::Identity(); // Initial Femur offset (used when resetting) [m]

  std::shared_ptr<mc_tasks::TransformTask> tibia_task_;
  std::shared_ptr<mc_tasks::TransformTask> femur_task_;
  bool continuous_ = false;
  bool measure_ = true;

  mc_trajectory::SequenceInterpolator<sva::PTransformd, PTransformInterpolator> tibiaOffsetInterpolator_,
      femurOffsetInterpolator_;
  double offsetDuration_ = 2;
  double tibiaOffsetInterpolationTime_ = 0;
  double femurOffsetInterpolationTime_ = 0;
};
