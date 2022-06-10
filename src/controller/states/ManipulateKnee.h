#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
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
  void play()
  {
    if(file_.tibiaRotationVector.empty())
    {
      mc_rtc::log::error("[{}] Cannot play, empty file", name());
      return;
    }
    play_ = true;
    measuredSamples_ = 0;
  }

  void stop()
  {
    play_ = false;
    saveResults();
  }

  void saveResults()
  {
    results_.write_csv(resultPath_);
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

  /// Returns true when all measurements have been taken
  bool measure(mc_control::fsm::Controller & ctl);

protected:
  ReadCSV file_;
  bool play_ = false;
  bool next_ = true;
  size_t iter_ = 0;
  size_t iterRate_ = 1;
  double translationTreshold_ = 1; ///< Convergence threshold on translation [mm]
  double rotationTreshold_ = 0.1; ///< Convergence threshold on rotation [deg]

  unsigned desiredSamples_ = 10;
  unsigned measuredSamples_ = 0;

  ResultHandler results_;
  std::string resultPath_ = "/tmp/BoneTagResults.csv";

  Eigen::Vector3d femurTranslation_ = Eigen::Vector3d::Zero(); ///< Desired joint translation in [mm]
  Eigen::Vector3d femurTranslationActual_ = Eigen::Vector3d::Zero(); ///< Current joint translation in [mm]
  Eigen::Vector3d minFemurTranslation_{-20, -20, -10}; ///< Min translation [mm]
  Eigen::Vector3d maxFemurTranslation_ = {20, 20, 10}; ///< Max translation [mm]

  Eigen::Vector3d tibiaTranslation_ = Eigen::Vector3d::Zero(); ///< Desired Joint translation in [mm]
  Eigen::Vector3d tibiaTranslationActual_ = Eigen::Vector3d::Zero(); ///< Current joint translation in [mm]
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

  std::shared_ptr<mc_tasks::TransformTask> tibia_task_;
  std::shared_ptr<mc_tasks::TransformTask> femur_task_;
};
