#include "controller.h"
#include "config.h"

namespace panda_prosthetics
{

PandaProsthetics::PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  gui()->addElement(
      {"Frames"}, mc_rtc::gui::Transform("Tibia", [this]() { return robot("panda_tibia").frame("Tibia").position(); }),
      mc_rtc::gui::Transform("Femur", [this]() { return robot("panda_femur").frame("Femur").position(); }),
      mc_rtc::gui::Transform("TibiaCalibration",
                             [this]() { return robot("panda_tibia").frame("TibiaCalibration").position(); }),
      mc_rtc::gui::Transform("FemurCalibration",
                             [this]() { return robot("panda_femur").frame("FemurCalibration").position(); }));
}

bool PandaProsthetics::run()
{
  return mc_control::fsm::Controller::run();
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
}

void PandaProsthetics::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

} // namespace panda_prosthetics
