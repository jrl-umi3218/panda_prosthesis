#include "controller.h"
#include "config.h"

namespace panda_prosthetics
{

PandaProsthetics::PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
}

bool PandaProsthetics::run()
{
  return mc_control::fsm::Controller::run();
}

void PandaProsthetics::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

} // namespace panda_prosthetics
