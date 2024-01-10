#include "RecordTrajectory.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/gui/StringInput.h>
#include <mc_rtc/io_utils.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "../include/utils.h"

void RecordTrajectory::start(mc_control::fsm::Controller & ctl)
{
  directory_ =
      static_cast<std::string>(ctl.config()("TrajectoryLoaders")("RecordedBrace")("directory")) + "/recorded_brace";
  auto etc_dir = static_cast<std::string>(ctl.config()("ETC_DIR"));
  loadDirectory(ctl);
  savedConfigPath_ = etc_dir + "/" + "panda_brace_saved.yaml";
  savedConfig_.load(savedConfigPath_);
  mc_rtc::log::info("Saved config is: {}", savedConfig_.dump(true, true));
  if(savedConfig_.has("RecordTrajectory"))
  {
    savedConfig_("RecordTrajectory")("selectedFile", fileName_);
  }

  if(fileName_.size())
  {
    loadFile(ctl, fileName_);
  }

  updateGUI(ctl);

  output("OK");
}

void RecordTrajectory::addPose(mc_control::fsm::Controller & ctl)
{
  poses.push_back(ctl.robot().frame("Femur").position()
                  * ctl.robot("brace_bottom_setup").frame("Tibia").position().inv());
  save();
  updateGUI(ctl);
}

void RecordTrajectory::loadFile(mc_control::fsm::Controller & ctl, const std::string & fileName)
{
  mc_rtc::log::info("[{}] Loading file {}", name(), fileName);
  fileName_ = fileName;
  mc_rtc::Configuration conf;
  conf.load(directory_ + "/" + fileName);
  poses = conf("poses", std::vector<sva::PTransformd>{});
  updateGUI(ctl);
}

void RecordTrajectory::loadDirectory(mc_control::fsm::Controller & ctl)
{
  files_ = get_all_filenames(directory_, ".yaml");
  mc_rtc::log::info("[{}] Looking for trajectory files in \"{}\"", name(), directory_);
  if(files_.size())
  {
    mc_rtc::log::info("[{}] Found trajectory files: {}", name(), mc_rtc::io::to_string(files_));
  }
  else
  {
    mc_rtc::log::warning("[{}] No trajectory file found in \"{}\" (expected extension .yaml)", name(), directory_);
  }
  updateGUI(ctl);
}

void RecordTrajectory::removePose(mc_control::fsm::Controller & ctl, size_t i)
{
  poses.erase(poses.begin() + i);
  save();
  updateGUI(ctl);
}

void RecordTrajectory::save()
{
  auto path = directory_ + "/" + fileName_;
  mc_rtc::Configuration conf;
  conf.add("poses", poses);
  conf.save(path);
  mc_rtc::log::success("[{}] Saved trajectory with {} poses to {}", name(), poses.size(), path);
  // set this file as the new default one
  if(!savedConfig_.has("RecordTrajectory"))
  {
    savedConfig_.add("RecordTrajectory", mc_rtc::Configuration{});
  }
  savedConfig_("RecordTrajectory").add("selectedFile", fileName_);
  savedConfig_.save(savedConfigPath_);
}

void RecordTrajectory::updateGUI(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({"RecordTrajectory"});

  ctl.gui()->addElement(this, {"RecordTrajectory"},
                        mc_rtc::gui::StringInput(
                            "New file name", []() { return "newfile.yaml"; },
                            [this, &ctl](const std::string & name)
                            {
                              fileName_ = name;
                              files_.push_back(fileName_);
                              poses.clear();
                              updateGUI(ctl);
                            }));

  if(fileName_.size())
  {
    ctl.gui()->addElement(this, {"RecordTrajectory"}, mc_rtc::gui::Button("Add Pose", [this, &ctl]() { addPose(ctl); }),
                          mc_rtc::gui::Button("Remove last pose",
                                              [this, &ctl]()
                                              {
                                                if(poses.size() >= 1)
                                                {
                                                  removePose(ctl, poses.size() - 1);
                                                }
                                              }),
                          mc_rtc::gui::Button("Done", [this]() { done_ = true; }));
  }

  ctl.gui()->addElement(this, {"RecordTrajectory"},
                        mc_rtc::gui::ComboInput(
                            "Trajectory", files_, [this]() { return fileName_; },
                            [this, &ctl](const std::string & name)
                            {
                              loadFile(ctl, name);
                              save();
                            }));

  for(size_t i = 0; i < poses.size(); ++i)
  {
    const auto & pose = poses[i];
    ctl.gui()->addElement(
        this, {"RecordTrajectory", "Poses", std::to_string(i)}, mc_rtc::gui::ElementsStacking::Vertical,
        mc_rtc::gui::ArrayLabel("translation [m]", [pose]() -> Eigen::Vector3d { return pose.translation(); }),
        mc_rtc::gui::ArrayLabel("rpy [deg]",
                                [pose]() -> Eigen::Vector3d {
                                  return mc_rbdyn::rpyFromMat(pose.rotation().inverse()) * 180 / mc_rtc::constants::PI;
                                }),
        mc_rtc::gui::Button("Remove", [this, &ctl, i]() { removePose(ctl, i); }));
  }
}

bool RecordTrajectory::run(mc_control::fsm::Controller & ctl)
{
  return done_;
}

void RecordTrajectory::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("RecordTrajectory", RecordTrajectory)
