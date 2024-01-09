#include "RecordTrajectory.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <SpaceVecAlg/SpaceVecAlg>

void RecordTrajectory::start(mc_control::fsm::Controller & ctl)
{
  auto directory = static_cast<std::string>(ctl.config()("TrajectoryLoaders")("RecordedBrace")("directory"));
  config_("fileName", fileName_);
  config_("overwrite", overwrite_);

  mc_rtc::Configuration conf;
  path = directory + "/" + "recorded_brace/" + fileName_;
  if(!overwrite_)
  {
    conf.load(path);
    poses = conf("poses");
  }
  auto addPose = [&, this]()
  {
    poses.push_back(ctl.robot().frame("Femur").position()
                    * ctl.robot("brace_bottom_setup").frame("Tibia").position().inv());
    save();
    updateGUI(ctl);
  };

  ctl.gui()->addElement(this, {"RecordTrajectory"}, mc_rtc::gui::Button("Add Pose", [addPose]() { addPose(); }),
                        mc_rtc::gui::Button("Done", [this]() { done_ = true; }),
                        mc_rtc::gui::Button("Remove last pose",
                                            [this, &ctl]()
                                            {
                                              if(poses.size() >= 1)
                                              {
                                                removePose(ctl, poses.size() - 1);
                                              }
                                            }));

  updateGUI(ctl);

  output("OK");
}

void RecordTrajectory::removePose(mc_control::fsm::Controller & ctl, size_t i)
{
  poses.erase(poses.begin() + i);
  save();
  updateGUI(ctl);
}

void RecordTrajectory::save()
{
  mc_rtc::Configuration conf;
  conf.add("poses", poses);
  conf.save(path);
  mc_rtc::log::success("Saved trajectory to {}", path);
}

void RecordTrajectory::updateGUI(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({"RecordTrajectory", "Poses"});
  for(size_t i = 0; i < poses.size(); ++i)
  {
    ctl.gui()->addElement(
        this, {"RecordTrajectory", "Poses", std::to_string(i)}, mc_rtc::gui::ElementsStacking::Vertical,
        mc_rtc::gui::Label("Pose", [i]() { return std::to_string(i); }),
        mc_rtc::gui::ArrayLabel("translation [m]" + std::to_string(i),
                                [this, i]() -> const Eigen::Vector3d & { return poses[i].translation(); }),
        mc_rtc::gui::ArrayLabel("rpy [deg]" + std::to_string(i),
                                [this, i]() -> const Eigen::Vector3d {
                                  return mc_rbdyn::rpyFromMat(poses[i].rotation().inverse()) * 180
                                         / mc_rtc::constants::PI;
                                }),
        mc_rtc::gui::Button("Remove", [this, &ctl, i]() { removePose(ctl, i); }));
  }
}

bool RecordTrajectory::run(mc_control::fsm::Controller & ctl)
{
  return done_;
}

void RecordTrajectory::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("RecordTrajectory", RecordTrajectory)
