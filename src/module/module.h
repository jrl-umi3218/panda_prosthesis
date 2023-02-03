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

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"PandaProsthesis::Femur", "PandaProsthesis::Tibia", "PandaBrace::Femur"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaProsthesis")
    if(n == "PandaProsthesis::Femur")
    {
      return new mc_robots::PandaProsthesisRobotModule("femur");
    }
    else if(n == "PandaProsthesis::Tibia")
    {
      return new mc_robots::PandaProsthesisRobotModule("tibia");
    }
    else if(n == "PandaBrace::Femur")
    {
      return new mc_robots::PandaBraceRobotModule();
    }
    else
    {
      mc_rtc::log::error("Panda module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
