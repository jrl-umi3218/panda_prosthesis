#pragma once
#include "module.h"

namespace mc_robots
{

/**
 * Robot module for the Panda femur with the top brace (brace_top_setup)
 * attached
 **/
template<bool DebugLog = false>
struct ROBOT_MODULE_API PandaBraceRobotModule : public mc_robots::PandaRobotModule
{
public:
  PandaBraceRobotModule();

protected:
  template<typename... Args>
  inline void log_info(Args &&... args)
  {
    if constexpr(DebugLog)
    {
      mc_rtc::log::info(std::forward<Args>(args)...);
    }
  }

  template<typename... Args>
  inline void log_success(Args &&... args)
  {
    if constexpr(DebugLog)
    {
      mc_rtc::log::success(std::forward<Args>(args)...);
    }
  }
};

} // namespace mc_robots
