#include "PlayTrajectory.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>

void PlayTrajectory::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::success("[{}] started", name());
  if(!ctl.datastore().has("Trajectories"))
  {
    mc_rtc::log::error("[{}] No trajectory to play", name());
    output("NoTrajectory");
  }
  else
  {
    auto & trajectories = ctl.datastore().get<std::vector<Trajectory>>("Trajectories");
    mc_rtc::log::info("[{}] Playing {} trajectories: {}", name(), trajectories.size(),
                      mc_rtc::io::to_string(trajectories, [](const Trajectory & traj) { return traj.name(); }));
    for(auto & traj : trajectories)
    {
      traj.update();
      auto config = mc_rtc::Configuration{};
      if(config_.has("TrajectoryPlayer") && config_("TrajectoryPlayer").has(traj.name()))
      {
        config = config_("TrajectoryPlayer")(traj.name());
        mc_rtc::log::info("config is {}", config.dump(true));
      }
      trajPlayers_.push_back(std::make_shared<TrajectoryPlayer>(ctl.solver(), traj, config));
    }
  }
  output("OK");
}

bool PlayTrajectory::run(mc_control::fsm::Controller & ctl)
{
  for(auto & trajPlayer : trajPlayers_)
  {
    trajPlayer->update(ctl.timeStep);
  }
  // mc_rtc::log::info("Finished: {}", finished());
  return finished();
}

void PlayTrajectory::teardown(mc_control::fsm::Controller & ctl)
{
  if(ctl.datastore().has("Trajectories"))
  {
    ctl.datastore().get<std::vector<Trajectory>>("Trajectories").clear();
  }
}

EXPORT_SINGLE_STATE("PlayTrajectory", PlayTrajectory)
