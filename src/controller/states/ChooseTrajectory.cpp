#include "ChooseTrajectory.h"
#include <mc_control/fsm/Controller.h>

void ChooseTrajectory::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::success("[{}] started", name());

  for(const auto & [name, config] : ctl.config()("TrajectoryLoaders", std::map<std::string, mc_rtc::Configuration>{}))
  {
    if(name == "BoneTag")
    {
      if(ctl.hasRobot("panda_femur") && ctl.hasRobot("panda_tibia"))
      { // If we have two robots allow loading bonetag trajectories
        // auto boneTag = std::make_unique<BoneTagTrajectoryLoader>();
        loaders_["BoneTagTrajectory"] = std::make_unique<BoneTagTrajectoryLoader>(
            ctl.robot("panda_tibia").frame("Tibia"), ctl.robot("panda_femur").frame("Femur"));
        loader_ = "BoneTagTrajectory";
        loaders_[loader_]->directory(config("directory"));
      }
      else
      {
        mc_rtc::log::error_and_throw("[{}] The BoneTagTrajectory loader requires robots panda_tibia and panda_femur");
      }
    }
    else if(name == "Brace")
    {
      if(ctl.hasRobot("panda_brace_femur") && ctl.hasRobot("brace_bottom_setup"))
      {
        loaders_["BraceTrajectory"] = std::make_unique<BraceTrajectoryLoader>(
            ctl.robot("panda_brace_femur").frame("Femur"), ctl.robot("brace_bottom_setup").frame("Tibia"));
        loader_ = "BraceTrajectory";
        loaders_[loader_]->directory(config("directory"));
        if(ctl.datastore().has("AtiDaq::removeForceSensorOffsets"))
        {
          mc_rtc::log::info("[AtiDaq] Removing force sensor offsets");
          ctl.datastore().call<void>("AtiDaq::removeForceSensorOffsets");
        }
      }
      else
      {
        mc_rtc::log::error_and_throw(
            "[{}] The BraceTrajectory loader requires robots panda_brace_femur and brace_bottom_setup");
      }
    }
    else
    {
      mc_rtc::log::error_and_throw("[{}] Unsupported trajectory loader {}", this->name(), name);
    }
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
                            [this, &ctl](const std::string & name)
                            {
                              auto & l = *loaders_[loader_];
                              mc_rtc::log::info("Adding to GUI {}", l.name());
                              l.addToGUI(*ctl.gui(), {"ChooseTrajectory"});
                            }),
                        mc_rtc::gui::Button("Play Trajectory",
                                            [this, &ctl]()
                                            {
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
