#include "CalibrateRelative.h"
#include <mc_control/fsm/Controller.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

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

  // 1- Suppose that the tibia is parallel to the floor
  // 2- Suppose that the robot femur axis has been placed at a known position X_tibia_femur w.r.t the tibia axis (manually)
  auto X_tibia_femur = config_("relativePose", sva::PTransformd::Identity()); // XXX accurate transform to be provided by Lea
  // 3- Move tibia position to match this known position 
  auto X_0_femur = ctl.robot().frame(frameName).position();
  auto X_0_tibia = X_tibia_femur.inv() * X_0_femur;
  auto X_tibia_base = ctl.robot(targetRobotName).posW() * ctl.robot(targetRobotName).frame(targetFrameName).position().inv();
  Eigen::Matrix3d mergeRPY = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(Eigen::Matrix3d::Identity(), X_0_tibia.rotation()); 
  X_0_tibia.rotation() = mergeRPY; 
  auto X_0_tibiaBase = X_tibia_base * X_0_tibia;
  ctl.robot(targetRobotName).posW(X_0_tibiaBase);

  // 4- Add task on femur to move it to a known transformation between tibia and femur 
  auto X_0_femurTarget = X_tibia_femur * X_0_tibia;
  transformTask_ = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot(robotName).frame(frameName), 5.0, 1000, 1000, X_0_femurTarget);
  transformTask_->reset();
  ctl.solver().addTask(transformTask_);

  output("OK");
}

bool CalibrateRelative::run(mc_control::fsm::Controller & ctl)
{
  return transformTask_->timeElapsed();
}

void CalibrateRelative::teardown(mc_control::fsm::Controller & ctl)
{

  ctl.solver().removeTask(transformTask_);
}

EXPORT_SINGLE_STATE("CalibrateRelative", CalibrateRelative)
