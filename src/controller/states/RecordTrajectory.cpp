#include "RecordTrajectory.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
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
  };

  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Add Pose", [addPose]() { addPose(); }),
                        mc_rtc::gui::Button("Save", [this]() { save_ = true; }));

  output("OK");
}

bool RecordTrajectory::run(mc_control::fsm::Controller & ctl)
{
  if(save_)
  {
    mc_rtc::Configuration conf;
    conf.add("poses", poses);
    conf.save(path);
    mc_rtc::log::success("Saved trajectory to {}", path);
    return true;
  }
  return false;
}

void RecordTrajectory::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("RecordTrajectory", RecordTrajectory)
