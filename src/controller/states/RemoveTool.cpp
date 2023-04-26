#include "RemoveTool.h"
#include <mc_control/fsm/Controller.h>



void RemoveTool::start(mc_control::fsm::Controller & ctl)
{
  //ctl.gui()->addElement(this, { "Remove Tool" },
    //    mc_rtc::gui::Button("Remove Tool", [this, &ctl]() {output("RemoveTool"); }));
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

  auto targetTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 2.0, 500.0);
  auto X_tibia_femur = sva::PTransformd(Eigen::Vector3d{ 0,0,0.1 });
  auto X_0_femur = X_tibia_femur * ctl.robot(robotName).frame(frameName).position();
  targetTask_->target(X_0_femur);
  ctl.solver().addTask(targetTask_);
  

  output("OK");
}

bool RemoveTool::run(mc_control::fsm::Controller & ctl)
{
    return true;
}

void RemoveTool::teardown(mc_control::fsm::Controller& ctl) {
  
    ctl.getPostureTask(config_("robot"))->reset();
    // XXX set high stiffness so that the robot stays here while selecting the trajectory
    // consider a better way
    ctl.getPostureTask(config_("robot"))->stiffness(100);
    ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("RemoveTool", RemoveTool)