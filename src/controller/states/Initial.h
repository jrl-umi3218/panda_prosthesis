#pragma once

#include <mc_control/fsm/State.h>
#include <mc_trajectory/SequenceInterpolator.h>

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
  double duration_ = 2;
  using PostureInterpolator = mc_trajectory::SequenceInterpolator<Eigen::VectorXd>;
  PostureInterpolator postureInterp_;
  bool pose_changed_ = false;
  bool load_ = true;
  bool reset_mbc_ = false;
  bool useJoints_ = true;
  std::string frame_;
  double saved_stiffness_ = 1.0;
  sva::PTransformd initial_pose_ = sva::PTransformd::Identity();
  bool done_ = false;
  double t_ = 0;
};
