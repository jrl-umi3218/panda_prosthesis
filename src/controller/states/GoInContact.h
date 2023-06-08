#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include "../include/TrajectoryLoader.h"

struct GoInContact : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  std::map<std::string, std::unique_ptr<TrajectoryLoader>> loaders_;
  std::string loader_;
  bool clicked = false;
  Eigen::DenseCoeffsBase<Eigen::Matrix<double, 3, 1>, 1>::Scalar target_force_z;

  std::shared_ptr<mc_tasks::TransformTask> transfoTask_;

  sva::MotionVecd velB_ = sva::MotionVecd::Zero();

  sva::MotionVecd damp_ = sva::MotionVecd::Zero();

  sva::MotionVecd stiff_ = sva::MotionVecd::Zero();
};
