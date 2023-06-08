#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include "../include/TrajectoryLoader.h"

struct RemoveTool : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  std::map<std::string, std::unique_ptr<TrajectoryLoader>> loaders_;
  std::string loader_;
  bool completed = false;
  bool pass = false;
  std::shared_ptr<mc_tasks::TransformTask> targetTask_;
  double err = 100;
};
