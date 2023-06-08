#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>

struct OffsetCalib : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  bool okep = false;
  bool go_next = false;
  std::shared_ptr<mc_tasks::TransformTask> offsetTask_;
  double err = 100;
};
