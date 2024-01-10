#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

struct RecordTrajectory : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addPose(mc_control::fsm::Controller & ctl, bool useReal = true);
  void removePose(mc_control::fsm::Controller & ctl, size_t i);
  void updateGUI(mc_control::fsm::Controller & ctl);
  void save();
  void loadDirectory(mc_control::fsm::Controller & ctl);
  void loadFile(mc_control::fsm::Controller & ctl, const std::string & fileName);

private:
  std::string directory_ = "";
  std::vector<std::string> files_;
  std::string fileName_ = "";
  std::vector<sva::PTransformd> poses{};
  std::string savedConfigPath_ = "";
  mc_rtc::Configuration savedConfig_;
  bool done_ = false;
};
