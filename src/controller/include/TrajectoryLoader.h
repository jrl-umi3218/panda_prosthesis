#pragma once
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/io_utils.h>
#include <mc_rbdyn/Robots.h>
#include <boost/filesystem.hpp>
#include "trajectory.h"
#include <string>
#include <variant>

/**
 * \brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
inline std::vector<std::string> get_all_filenames(boost::filesystem::path const & root, std::string const & ext)
{
  std::vector<std::string> paths;

  if(boost::filesystem::exists(root) && boost::filesystem::is_directory(root))
  {
    for(auto const & entry : boost::filesystem::recursive_directory_iterator(root))
    {
      if(boost::filesystem::is_regular_file(entry) && entry.path().extension() == ext)
        paths.emplace_back(entry.path().filename().c_str());
    }
  }

  return paths;
}

struct TrajectoryLoader
{
  TrajectoryLoader() {}

  void directory(const std::string & directory)
  {
    directory_ = directory;
    loadDirectory();
  }

  virtual void load(const std::string & directory, const std::string & csv)
  {
    selectedFile_ = csv;
    // static_cast<Derived *>(this)->load(directory, csv);
  }

  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
  {
    category.push_back(name_);
    using namespace mc_rtc::gui;
    gui.addElement(this, category,
                   mc_rtc::gui::ComboInput(
                       "Trajectory", files_, [this]() { return selectedFile_; },
                       [this](const std::string & name) {
                         selectedFile_ = name;
                         load(directory_, name);
                       }));
  }

  virtual void removeFromGUI(mc_rtc::gui::StateBuilder & gui)
  {
    using namespace mc_rtc::gui;
    gui.removeElements(this);
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  virtual std::vector<Trajectory> trajectories() const = 0;

protected:
  void loadDirectory()
  {
    files_ = get_all_filenames(directory_, ".csv");
    mc_rtc::log::info("[{}] Looking for trajectory files in \"{}\"", name_, directory_);
    if(files_.size())
    {
      mc_rtc::log::info("Found trajectory files: {}", mc_rtc::io::to_string(files_));
    }
    else
    {
      mc_rtc::log::warning("No trajectory file found in \"{}\" (expected extension .csv)", directory_);
    }
  }

protected:
  std::string directory_;
  std::vector<std::string> files_;
  std::string selectedFile_;
  std::string name_{"TrajectoryLoader"};
};

struct BoneTagTrajectoryLoader : public TrajectoryLoader
{
  BoneTagTrajectoryLoader(const mc_rbdyn::RobotFrame & tibiaFrame, const mc_rbdyn::RobotFrame & femurFrame)
      : trajTibia("Tibia Trajectory", tibiaFrame), trajFemur("Femur Trajectory", femurFrame)
  {
    name_ = "BoneTagTrajectoryLoader";
  }

  void load(const std::string & directory, const std::string & csv) override
  {
    TrajectoryLoader::load(directory, csv);
    trajFemur.clear();
    trajTibia.clear();
    trajFemur.loadPoseFromCSV(directory + "/" + csv, "femur_tangage", "femur_roulis", "femur_lacet", "femur_x",
                              "femur_y", "femur_z");
    trajTibia.loadPoseFromCSV(directory + "/" + csv, "tibia_tangage", "tibia_roulis", "tibia_lacet", "tibia_x",
                              "tibia_y", "tibia_z");
    trajTibia.update();
    trajFemur.update();
    mc_rtc::log::info("Loaded BoneTag trajectory {} with {} poses", csv, trajFemur.poses().size());
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category) override
  {
    using namespace mc_rtc::gui;
    TrajectoryLoader::addToGUI(gui, category);
    category.push_back(name_);
    trajTibia.addToGUI(gui, category);
    trajFemur.addToGUI(gui, category);
  }

  const Trajectory & tibiaTrajectory() const
  {
    return trajTibia;
  }

  const Trajectory & femurTrajectory() const
  {
    return trajFemur;
  }

  std::vector<Trajectory> trajectories() const override
  {
    return {trajTibia, trajFemur};
  }

protected:
  Trajectory trajTibia;
  Trajectory trajFemur;
};
