#pragma once

#include <mc_control/fsm/State.h>

struct CalibrateRelative : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;
};
