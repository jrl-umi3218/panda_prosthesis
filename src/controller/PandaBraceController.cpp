#include "PandaBraceController.h"
#include <mc_rtc/io_utils.h>
#include "config.h"

namespace panda_prosthetics
{

PandaBraceController::PandaBraceController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  auto & gui = *this->gui();
  gui.addElement({"PandaBrace", "Frames"},
                    mc_rtc::gui::Transform("Femur", [this]() { return robot().frame("Femur").position(); }),
  		    mc_rtc::gui::Transform("Tibia", [this]() { return robot("brace_bottom_setup").frame("Tibia").position(); }));

  gui.addElement({"PandaBrace", "Forces"},
   mc_rtc::gui::Label("Details",
     [this]() { return std::string{"Forces without gravity expressed in the reference frame"}; }));
  gui.addElement({"PandaBrace", "Forces", robot().name()},
                    mc_rtc::gui::Force("FemurWrench",
                      [this]() { return robot().frame("Femur").wrench(); },
                      [this]() { return robot().frame("Femur").position(); }));
  gui.addElement({"PandaBrace", "Forces", "brace_bottom_setup"},
                    mc_rtc::gui::Force("TibiaWrench",
                      [this]() { return robot("brace_bottom_setup").frame("Tibia").wrench(); },
                      [this]() { return robot("brace_bottom_setup").frame("Tibia").position(); }));
  logger().addLogEntry("controlRobot_Frame_Femur",
   [this]() { return robot().frame("Femur").position(); });
  logger().addLogEntry("realRobot_Frame_Femur",
   [this]() { return realRobot().frame("Femur").position(); });
}

bool PandaBraceController::run()
{
  return mc_control::fsm::Controller::run();
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
}

void PandaBraceController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("PandaBrace", panda_prosthetics::PandaBraceController)

} // namespace panda_prosthetics
