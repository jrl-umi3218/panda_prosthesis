#include "ChooseTrajectory.h"
#include <mc_control/fsm/Controller.h>

void ChooseTrajectory::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::success("[{}] started", name());
  if(ctl.hasRobot("panda_femur") && ctl.hasRobot("panda_tibia"))
  { // If we have two robots allow loading bonetag trajectories
    auto boneTag = BoneTagTrajectoryLoader{};
    boneTag.directory(ctl.config()("TrajectoryLoaders")("BoneTag")("directory"));
    loaders_["BoneTagTrajectory"] = boneTag;
    loader_ = "BoneTagTrajectory";
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
  for(auto & loader : loaders_)
  {
    keys.push_back(loader.first);
    std::visit(
        [&ctl](auto & l) {
          mc_rtc::log::info("Adding to GUI {}", l.name());
          l.addToGUI(*ctl.gui(), {"ChooseTrajectory"});
        },
        loader.second);
  }

  ctl.gui()->addElement({"ChooseTrajectory"},
                        mc_rtc::gui::ComboInput(
                            "Trajectory Loader", keys, [this]() { return loader_; },
                            [this, &ctl](const std::string & name) {
                              loader_ = name;
                              std::visit(
                                  [&ctl](auto & l) {
                                    mc_rtc::log::info("Adding to GUI {}", l.name());
                                    l.addToGUI(*ctl.gui(), {"ChooseTrajectory"});
                                  },
                                  loaders_[loader_]);
                            }),
                        mc_rtc::gui::Button("Play Trajectory", [this, &ctl]() {
                          std::visit(
                              [this, &ctl](auto & l) {
                                bool valid = true;
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
                              },
                              loaders_[loader_]);
                        }));
  output("OK");
};

bool ChooseTrajectory::run(mc_control::fsm::Controller & ctl)
{
  return ctl.datastore().get<std::vector<Trajectory>>("Trajectories").size();
}

void ChooseTrajectory::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("ChooseTrajectory", ChooseTrajectory)
