#pragma once

#include <mc_control/fsm/State.h>

struct Initial : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void load(mc_control::fsm::Controller & ctl);

private:
  std::vector<std::string> category_{};
  std::string robot_;
  std::string etc_file_;
  bool pose_changed_ = false;
  bool load_ = true;
  bool reset_mbc_ = true;
  std::string frame_;
  double saved_stiffness_ = 1.0;
  sva::PTransformd initial_pose_ = sva::PTransformd::Identity();
  sva::PTransformd default_pose_ = sva::PTransformd::Identity();
};
