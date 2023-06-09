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

  // Suppose that the femur and calibration tool are in perfect contact
  // and that the relative position between the femur and tibia frames is fully known in that configuration
  auto X_femur_tibia = config_("femurToTibia", sva::PTransformd::Identity());
  auto X_0_femur = ctl.robot(robotName).frame(frameName).position();
  auto X_0_tibia = X_femur_tibia * X_0_femur;

  // Apply vertical offset w.r.t Link6 frame (after calib)
  auto X_0_refFrame = ctl.robot(refRobotName).frame(refFrameName).position();
  auto X_0_targetFrame = ctl.robot(targetRobotName).frame(targetFrameName).position();
  auto offset_link6 = config_("offsetLink6", sva::PTransformd::Identity());
  auto X_tibia_refFrame = X_0_refFrame * X_0_targetFrame.inv();
  auto X_0_refFrameOffset = offset_link6 * X_tibia_refFrame * X_0_tibia;
  X_0_tibia = X_tibia_refFrame.inv() * X_0_refFrameOffset;

  // Get relative transformation between tibia frame and brace_bottom_setup base link
  auto X_0_tibiaF = ctl.robot(targetRobotName).frame(targetFrameName).position();
  auto X_0_baseF = ctl.robot(targetRobotName).posW();
  auto X_base_tibia = X_0_tibiaF * X_0_baseF.inv();

  auto X_0_base = X_base_tibia.inv() * X_0_tibia;
  ctl.robot(targetRobotName).posW(X_0_base);

  output("OK");
}

bool CalibrateRelative::run(mc_control::fsm::Controller & ctl)
{
  return true;
}

void CalibrateRelative::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("CalibrateRelative", CalibrateRelative)
