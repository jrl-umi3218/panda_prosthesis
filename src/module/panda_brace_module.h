#pragma once
#include "module.h"

namespace mc_robots
{

/**
 * Robot module for the Panda femur with the top brace (brace_top_setup)
 * attached
 **/
struct ROBOT_MODULE_API PandaBraceRobotModule : public mc_robots::PandaRobotModule
{
public:
  PandaBraceRobotModule();
};

} // namespace mc_robots
