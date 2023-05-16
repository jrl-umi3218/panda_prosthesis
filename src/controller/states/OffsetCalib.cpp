#include "OffsetCalib.h"
#include <mc_control/fsm/Controller.h>



void OffsetCalib::start(mc_control::fsm::Controller& ctl)
{
    
   
    ctl.gui()->addElement(this, { "offset" },
        mc_rtc::gui::Button("offset_calib", [this, &ctl]() {
            auto robotName = static_cast<std::string>(config_("robot"));
            auto frameName = static_cast<std::string>(config_("frame"));
            if (!ctl.hasRobot(robotName))
            {
                mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), robotName);
            }
            if (!ctl.robot(robotName).hasFrame(frameName))
            {
                mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), frameName, robotName);
            }

            auto targetRobotName = static_cast<std::string>(config_("target_robot"));
            auto targetFrameName = static_cast<std::string>(config_("target_frame"));

            if (!ctl.hasRobot(targetRobotName))
            {
                mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), targetRobotName);
            }
            if (!ctl.robot(targetRobotName).hasFrame(targetFrameName))
            {
                mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), targetFrameName, targetRobotName);
            }
            offsetTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 2.0, 500.0);
            auto X_tibia_femur = sva::PTransformd(Eigen::Vector3d{ 0.01,0,0});
            auto X_0_femur = X_tibia_femur * ctl.robot(robotName).frame(frameName).position();
            offsetTask_->target(X_0_femur);

            
            okep = true;
            
            ctl.solver().addTask(offsetTask_);
            mc_rtc::log::info("Offset calib Ok");
            output("OK"); }),
        mc_rtc::gui::Button("Pass", [this, &ctl]() {
                go_next = true;
            }));
            
}

bool OffsetCalib::run(mc_control::fsm::Controller & ctl)
{   

    if (okep)
	{
        if (!go_next)
        {
           
        }
        if (go_next) {
            return true;
        }
    }
    return false;
}

void OffsetCalib::teardown(mc_control::fsm::Controller& ctl) {
    ctl.solver().removeTask(offsetTask_);
    ctl.getPostureTask(config_("robot"))->reset();
    // XXX set high stiffness so that the robot stays here while selecting the trajectory
    // consider a better way
    ctl.getPostureTask(config_("robot"))->stiffness(100);
    ctl.getPostureTask(config_("robot"))->weight(10);
    //ctl.solver().removeTask(targetTask_);
    ctl.gui()->removeElements(this);

   
}

EXPORT_SINGLE_STATE("OffsetCalib", OffsetCalib)
