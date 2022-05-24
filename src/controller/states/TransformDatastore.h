#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>

struct TransformDatastore : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  std::shared_ptr<mc_tasks::TransformTask> task_;
  std::string datastorePose_ = "";
  bool continuous_ = false;
  bool save_ = true;
  std::string robot_;
  mc_rtc::Configuration criteria_config_;
  mc_control::CompletionCriteria criteria_;
  sva::PTransformd X_0_target_ = sva::PTransformd::Identity();
};
