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
    auto trajectories = ctl.datastore().get<std::vector<Trajectory>>("Trajectories");
    ctl.datastore().get<std::vector<Trajectory>>("Trajectories").clear();
    mc_rtc::log::info("[{}] Playing {} trajectories: {}", name(), trajectories.size(),
                      mc_rtc::io::to_string(trajectories, [](const Trajectory & traj) { return traj.name(); }));
    for(auto traj : trajectories)
    {
      traj.update();
      auto config = mc_rtc::Configuration{};
      if(config_.has("TrajectoryPlayer") && config_("TrajectoryPlayer").has(traj.name()))
      {
        config = config_("TrajectoryPlayer")(traj.name());
        mc_rtc::log::info("config is {}", config.dump(true));
      }
      traj.interpolateInitialPose(traj.frame().position(), traj.frame().velocity());
      auto trajPlayer = std::make_shared<TrajectoryPlayer>(ctl.solver(), traj, config);
      trajPlayer->addToGUI(*ctl.gui(), {"PlayTrajectory"});
      trajPlayers_.push_back(trajPlayer);
    }
  }
  ctl.gui()->addElement(this, {"PlayTrajectory"},
                        mc_rtc::gui::Button("Reverse Trajectories",
                                            [this, &ctl]() {
                                              auto reversedTrajectories = std::vector<Trajectory>{};
                                              for(auto & trajPlayer : trajPlayers_)
                                              {
                                                auto traj = trajPlayer->trajectory();
                                                traj.reverse();
                                                reversedTrajectories.push_back(traj);
                                              }
                                              ctl.datastore().get<std::vector<Trajectory>>("Trajectories") =
                                                  reversedTrajectories;
                                            }),
                        mc_rtc::gui::Button("Next",
                                            [this]() {
                                              output("PlayTrajectory");
                                              next_ = true;
                                            }),
                        mc_rtc::gui::Button("ChooseTrajectory", [this]() {
                          output("ChooseTrajectory");
                          next_ = true;
                        }));
  output("OK");
}

bool PlayTrajectory::run(mc_control::fsm::Controller & ctl)
{
  for(auto & trajPlayer : trajPlayers_)
  {
    trajPlayer->update(ctl.timeStep);
  }
  return finished() && next_;
}

void PlayTrajectory::teardown(mc_control::fsm::Controller & ctl)
{
  for(auto & trajPlayer : trajPlayers_)
  {
    trajPlayer->removeFromGUI(*ctl.gui(), {"PlayTrajectory"});
  }
  trajPlayers_.clear();

  ctl.gui()->removeCategory({"PlayTrajectory"});
}

EXPORT_SINGLE_STATE("PlayTrajectory", PlayTrajectory)
