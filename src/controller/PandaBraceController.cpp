#include "PandaBraceController.h"
#include <mc_rtc/io_utils.h>
#include "config.h"

namespace panda_prosthetics
{

PandaBraceController::PandaBraceController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  gui()->addElement(
      {"Frames"}, mc_rtc::gui::Transform("Femur", [this]() { return robot().frame("Femur").position(); }),
      mc_rtc::gui::Transform("FemurCalibration", [this]() { return robot().frame("FemurCalibration").position(); }));
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
