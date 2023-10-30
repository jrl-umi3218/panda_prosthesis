#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>

struct RemoveTool : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  bool completed = false;
  bool pass = false;
  std::shared_ptr<mc_tasks::TransformTask> targetTask_;
  double err = 100;
  std::vector<std::string> category_{};
};
