#include "ChooseTrajectory.h"
#include <mc_control/fsm/Controller.h>

void ChooseTrajectory::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::success("[{}] started", name());
  if(ctl.hasRobot("panda_femur") && ctl.hasRobot("panda_tibia"))
  { // If we have two robots allow loading bonetag trajectories
    // auto boneTag = std::make_unique<BoneTagTrajectoryLoader>();
    loaders_["BoneTagTrajectory"] = std::make_unique<BoneTagTrajectoryLoader>(ctl.robot("panda_tibia").frame("Tibia"),
                                                                              ctl.robot("panda_femur").frame("Femur"));
    loader_ = "BoneTagTrajectory";
    loaders_[loader_]->directory(ctl.config()("TrajectoryLoaders")("BoneTag")("directory"));
  }
  else
  { // Only one robot
    mc_rtc::log::warning("We have only one robot");
  }

  if(loaders_.empty())
  {
    mc_rtc::log::error_and_throw("No valid loader found");
  }

  if(!ctl.datastore().has("Trajectories"))
  {
    ctl.datastore().make<std::vector<Trajectory>>("Trajectories");
  }

  auto keys = std::vector<std::string>{};
  for(auto & [name, loader] : loaders_)
  {
    keys.push_back(name);
    mc_rtc::log::info("Adding to GUI {}", name);
    loader->addToGUI(*ctl.gui(), {"ChooseTrajectory"});
  }

  ctl.gui()->addElement(this, {"ChooseTrajectory"},
                        mc_rtc::gui::ComboInput(
                            "Trajectory Loader", keys, [this]() { return loader_; },
                            [this, &ctl](const std::string & name) {
                              auto & l = *loaders_[loader_];
                              mc_rtc::log::info("Adding to GUI {}", l.name());
                              l.addToGUI(*ctl.gui(), {"ChooseTrajectory"});
                            }),
                        mc_rtc::gui::Button("Play Trajectory", [this, &ctl]() {
                          bool valid = true;
                          auto & l = *loaders_[loader_];
                          for(const auto & traj : l.trajectories())
                          {
                            valid = traj.poses().size();
                            if(!valid)
                            {
                              mc_rtc::log::error("Cannot play empty trajectory");
                              return;
                            }
                          }
                          ctl.datastore().assign("Trajectories", l.trajectories());
                        }));
  output("OK");
};

bool ChooseTrajectory::run(mc_control::fsm::Controller & ctl)
{
  return ctl.datastore().get<std::vector<Trajectory>>("Trajectories").size();
}

void ChooseTrajectory::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({"ChooseTrajectory"});
  for(auto & [name, loader] : loaders_)
  {
    loader->removeFromGUI(*ctl.gui());
  }
}

EXPORT_SINGLE_STATE("ChooseTrajectory", ChooseTrajectory)
