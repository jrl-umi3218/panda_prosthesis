#include "OffsetCalib.h"
#include <mc_control/fsm/Controller.h>

void OffsetCalib::start(mc_control::fsm::Controller & ctl)
{
  auto robotName = static_cast<std::string>(config_("robot"));
  auto frameName = static_cast<std::string>(config_("frame"));
  if(!ctl.hasRobot(robotName))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), robotName);
  }
  if(!ctl.robot(robotName).hasFrame(frameName))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), frameName, robotName);
  }

  auto refRobotName = static_cast<std::string>(config_("refRobot"));
  auto refFrameName = static_cast<std::string>(config_("refFrame"));

  if(!ctl.hasRobot(refRobotName))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), refRobotName);
  }
  if(!ctl.robot(refRobotName).hasFrame(refFrameName))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), refFrameName, refRobotName);
  }
  offsetTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 20.0, 1000.0);
  offsetTask_->reset();
  auto X_0_refFrame = ctl.robot(refRobotName).frame(refFrameName).position();
  auto X_0_targetFrame = ctl.robot(robotName).frame(frameName).position();
  auto X_refFrame_targetFrame = X_0_targetFrame * X_0_refFrame.inv(); // current pose of target wrt to ref frame
  auto offset = config_("offset", sva::PTransformd::Identity());
  auto X_0_target = X_refFrame_targetFrame * offset * X_0_refFrame;
  offsetTask_->target(X_0_target);

  crit.configure(*offsetTask_, ctl.timeStep, config_("completion"));
  ctl.solver().addTask(offsetTask_);
  output("OK");
}

bool OffsetCalib::run(mc_control::fsm::Controller & ctl)
{
  return crit.completed(*offsetTask_);
}

void OffsetCalib::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(offsetTask_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("OffsetCalib", OffsetCalib)
