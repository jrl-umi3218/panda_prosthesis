#pragma once

#include <mc_control/fsm/State.h>

struct Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool pose_changed_ = false;
  sva::PTransformd initial_pose_panda_femur_;
  sva::PTransformd initial_pose_panda_tibia_;
  std::vector<std::vector<double>> initial_joints_panda_femur_;
  std::vector<std::vector<double>> initial_joints_panda_tibia_;  
};
