#include "RemoveTool.h"
#include <mc_control/fsm/Controller.h>

void RemoveTool::start(mc_control::fsm::Controller & ctl)
{
  config_("category", category_);
  ctl.gui()->addElement(
      this, category_,
      mc_rtc::gui::Button(
          "RemoveTool",
          [this, &ctl]()
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
            targetTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 2.0, 500.0);
            auto X_tibia_femur = sva::PTransformd(Eigen::Vector3d{0, 0, 0.1});
            auto X_0_femur = X_tibia_femur * ctl.robot(robotName).frame(frameName).position();
            targetTask_->target(X_0_femur);

            ctl.getPostureTask(config_("robot"))->weight(1);
            completed = true;
            ctl.solver().addTask(targetTask_);
            mc_rtc::log::info("RemoveTool Ok");
          }),
      mc_rtc::gui::Button("Pass", [this, &ctl]() { pass = true; }));
  output("OK");
}

bool RemoveTool::run(mc_control::fsm::Controller & ctl)
{
  if(pass)
  {
    return true;
  }
  return false;
}

void RemoveTool::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(targetTask_);
  ctl.getPostureTask(config_("robot"))->reset();
  // XXX set high stiffness so that the robot stays here while selecting the trajectory
  // consider a better way
  ctl.getPostureTask(config_("robot"))->stiffness(100);
  ctl.getPostureTask(config_("robot"))->weight(10);
  // ctl.solver().removeTask(targetTask_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("RemoveTool", RemoveTool)
