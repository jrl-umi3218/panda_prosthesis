#include "RecordTrajectory.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>

void RecordTrajectory::start(mc_control::fsm::Controller & ctl)
{
  auto directory = static_cast<std::string>(ctl.config()("TrajectoryLoaders")("RecordedBrace")("directory"));
  config_("fileName", fileName_);
  config_("overwrite", overwrite_);

  std::vector<sva::PTransformd> poses;
  mc_rtc::Configuration conf;
  auto path = directory + "/" + "recorded_brace/" + fileName_;
  mc_rtc::log::info("Loading recorded trajectory from {}", path);
  if(!overwrite_)
  {
    conf.load(path);
    poses = conf("poses");
  }

  poses.push_back(ctl.robot().frame("Femur").position()
                  * ctl.robot("brace_bottom_setup").frame("Tibia").position().inv());

  conf.add("poses", poses);
  conf.save(path);

  output("OK");
}

bool RecordTrajectory::run(mc_control::fsm::Controller & ctl)
{
  return true;
}

void RecordTrajectory::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("RecordTrajectory", RecordTrajectory)
