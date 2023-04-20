#include "GoInContact.h"
#include <mc_control/fsm/Controller.h>



void GoInContact::start(mc_control::fsm::Controller& ctl)
{
 
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

    auto targetTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 1.0, 500.0);
    auto X_femur_contact = sva::PTransformd(Eigen::Vector3d{ 0,0,-0.2 }) ;
    auto X_transfo_axe = ctl.robot(targetRobotName).frame(targetFrameName).position();
    auto X_0_femur = X_femur_contact * X_transfo_axe;
    targetTask_->target(X_0_femur);
    ctl.solver().addTask(targetTask_);

 
    ctl.gui()->addElement(this, { "GoInContact" },
        mc_rtc::gui::Button("Go to Contact", [this, &ctl]() {output("GoInContact"); }));

    
    output("OK");
    
}

bool GoInContact::run(mc_control::fsm::Controller& ctl)
{
    return true;
}

void GoInContact::teardown(mc_control::fsm::Controller& ctl) {
    ctl.getPostureTask(config_("robot"))->reset();
    // XXX set high stiffness so that the robot stays here while selecting the trajectory
    // consider a better way
    ctl.getPostureTask(config_("robot"))->stiffness(100);
   
    ctl.gui()->removeElements(this);
   
}

EXPORT_SINGLE_STATE("GoInContact", GoInContact)