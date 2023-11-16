#pragma once

#include <mc_control/fsm/State.h>

struct RecordTrajectory : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string fileName_ = "default.yaml";
  bool overwrite_ = false;
  std::string path;
  std::vector<sva::PTransformd> poses;
  bool save_ = false;
};
