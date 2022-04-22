#pragma once

#include <mc_control/fsm/State.h>

struct Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);
  void save(mc_control::fsm::Controller & ctl);

private:
  std::vector<std::string> category_{};
  std::string robot_;
  std::string etc_file_;
  bool pose_changed_ = false;
  sva::PTransformd initial_pose_ = sva::PTransformd::Identity();
  sva::PTransformd default_pose_ = sva::PTransformd::Identity();
};
