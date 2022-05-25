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
  Eigen::Vector3d rotation_ = Eigen::Vector3d::Zero(); ///< Joint rotation in [deg]
  Eigen::Vector3d min_{-20, -10, -10};
  Eigen::Vector3d max_ = {20, 10, 10};
  double percentFemur = 0.8;
  std::shared_ptr<mc_tasks::TransformTask> tibia_task_;
  std::shared_ptr<mc_tasks::TransformTask> femur_task_;
  sva::PTransformd tibia_error_ = sva::PTransformd::Identity();
  sva::PTransformd femur_error_ = sva::PTransformd::Identity();
  sva::PTransformd tibia_angle_ = sva::PTransformd::Identity();
  sva::PTransformd femur_angle_ = sva::PTransformd::Identity();
};
