#include "CalibrateRelative.h"
#include <mc_control/fsm/Controller.h>

void CalibrateRelative::start(mc_control::fsm::Controller & ctl)
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

  auto targetRobotName = static_cast<std::string>(config_("target_robot"));
  auto targetFrameName = static_cast<std::string>(config_("target_frame"));
  if(!ctl.hasRobot(targetRobotName))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), targetRobotName);
  }
  if(!ctl.robot(targetRobotName).hasFrame(targetFrameName))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), targetFrameName, targetRobotName);
  }

  auto X_frame_targetFrame = config_("relativePose", sva::PTransformd::Identity());
  auto X_0_frame = ctl.robot(robotName).frame(frameName).position();
  auto X_0_targetFrame = ctl.robot(targetRobotName).frame(targetFrameName).position();

  auto X_0_targetRobotPosW =
      ctl.robot(targetRobotName).posW() * X_0_targetFrame.inv() * X_frame_targetFrame * X_0_frame;
  ctl.robot(targetRobotName).posW(X_0_targetRobotPosW);

  output("OK");
}

bool CalibrateRelative::run(mc_control::fsm::Controller & ctl)
{
  return true;
}

void CalibrateRelative::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("CalibrateRelative", CalibrateRelative)
