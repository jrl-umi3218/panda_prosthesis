#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_tasks/ImpedanceTask.h>
#include "../include/trajectory.h"

struct TrajectoryPlayer;

struct PlayTrajectory : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  bool finished() const;

protected:
  std::vector<std::shared_ptr<TrajectoryPlayer>> trajPlayers_;
  bool next_ = false;
};
