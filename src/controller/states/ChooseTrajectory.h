#pragma once

#include <mc_control/fsm/State.h>
#include "../include/TrajectoryLoader.h"

struct ChooseTrajectory : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  std::map<std::string, TrajectoryLoaders> loaders_;
  std::string loader_;
};
