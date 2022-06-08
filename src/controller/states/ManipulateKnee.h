#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>

struct ManipulateKnee : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  Eigen::Vector3d femurTranslation_ = Eigen::Vector3d::Zero(); ///< Joint translation in [mm]
  Eigen::Vector3d minFemurTranslation_{-20, -20, -10}; ///< Min translation [mm]
  Eigen::Vector3d maxFemurTranslation_ = {20, 20, 10}; ///< Max translation [mm]

  Eigen::Vector3d tibiaTranslation_ = Eigen::Vector3d::Zero(); ///< Joint translation in [mm]
  Eigen::Vector3d minTibiaTranslation_{-20, -20, -10}; ///< Min translation [mm]
  Eigen::Vector3d maxTibiaTranslation_ = {20, 20, 10}; ///< Max translation [mm]
                                                       ///
  Eigen::Vector3d tibiaRotation_ = Eigen::Vector3d::Zero(); ///< Joint rotation in [deg]
  Eigen::Vector3d minTibiaRotation_{-20, -10, -10}; ///< Min Allowed Rotation [deg]
  Eigen::Vector3d maxTibiaRotation_ = {20, 10, 10}; ///< Max Allowed Rotation [deg]

  Eigen::Vector3d femurRotation_ = Eigen::Vector3d::Zero(); ///< Joint rotation in [deg]
  Eigen::Vector3d minFemurRotation_{-20, -10, -10}; ///< Min Allowed Rotation [deg]
  Eigen::Vector3d maxFemurRotation_ = {20, 10, 10}; ///< Max Allowed Rotation [deg]

  std::shared_ptr<mc_tasks::TransformTask> tibia_task_;
  std::shared_ptr<mc_tasks::TransformTask> femur_task_;
  sva::PTransformd tibia_error_ = sva::PTransformd::Identity();
  sva::PTransformd femur_error_ = sva::PTransformd::Identity();
  sva::PTransformd tibia_angle_ = sva::PTransformd::Identity();
  sva::PTransformd femur_angle_ = sva::PTransformd::Identity();
};
