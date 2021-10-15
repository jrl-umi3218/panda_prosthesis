#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/CompletionCriteria.h>
#include <mc_tasks/SurfaceTransformTask.h>

struct GoRelative : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::SurfaceTransformTask> surfaceTask_;
  sva::PTransformd target_ = sva::PTransformd::Identity();
  mc_rtc::Configuration criteria_config_;
  mc_control::CompletionCriteria criteria_;
  bool has_target_ = false;
  std::string surface_ = {};
  std::string robot_ = {};
  double stiffness_ = 1;
  double weight_ = 100;
};
