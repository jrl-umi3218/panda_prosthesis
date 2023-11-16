#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

namespace panda_prosthetics
{

struct PandaProsthetics : public mc_control::fsm::Controller
{
  PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;
};

} // namespace panda_prosthetics
