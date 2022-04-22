#include "controller.h"
#include "config.h"

namespace panda_prosthetics
{

PandaProsthetics::PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  // TODO in mc_rtc: support loading rsdf from multiple directories defined in the robot module
  auto load_rsdf = [this](const std::string & robot_name) {
    auto & r = robot(robot_name);
    const auto rsdf_dir = std::string{panda_prosthesis::rsdf_DIR} + "/" + robot_name;
    mc_rtc::log::info("[{}] loading additional surfaces for robot {} from {}", this->name_, robot_name, rsdf_dir);
    r.loadRSDFFromDir(rsdf_dir);
    gui()->data()("surfaces").add(r.name(), r.availableSurfaces());
  };

  load_rsdf("panda_femur");
  load_rsdf("panda_tibia");
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
