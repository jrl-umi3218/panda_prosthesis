#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

#include <mc_panda/panda.h>

namespace mc_robots
{

struct ROBOT_MODULE_API PandaProsthesisRobotModule : public mc_robots::PandaRobotModule
{
public:
  PandaProsthesisRobotModule(const std::string & prosthesis);
};

} // namespace mc_robots
