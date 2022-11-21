#pragma once
#include <mc_rtc/gui.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <optional>

struct Trajectory
{
  /**
   * @brief Load pose (rotation/translation) from a CSV file
   *
   * @param csv CSV file from which to load
   * @param tangage
   * @param roulis
   * @param lacet
   * @param tx
   * @param ty
   * @param tz
   */
  void loadPoseFromCSV(const std::string & csv,
                       const std::string & tangage,
                       const std::string & roulis,
                       const std::string & lacet,
                       const std::string & tx,
                       const std::string & ty,
                       const std::string & tz);

  void loadVelocityFromCSV(const std::string & csv,
                           const std::string & cx,
                           const std::string & cy,
                           const std::string & cz,
                           const std::string & fx,
                           const std::string & fy,
                           const std::string & fz)
  {
  }

  void loadForceFromCSV(const std::string & csv,
                        const std::string & cx,
                        const std::string & cy,
                        const std::string & cz,
                        const std::string & fx,
                        const std::string & fy,
                        const std::string & fz)
  {
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
  {
    category.push_back(name_);
    using namespace mc_rtc::gui;
    gui.addElement(this, category, mc_rtc::gui::Label("Robot", [this]() { return robot_; }),
                   mc_rtc::gui::Label("Frame", [this]() { return robotFrame_; }),
                   mc_rtc::gui::Label("Poses", [this]() { return poses_.size(); }),
                   mc_rtc::gui::Label("Velocities", [this]() { return velocities_ ? velocities_->size() : 0; }),
                   mc_rtc::gui::Label("Forces", [this]() { return forces_ ? forces_->size() : 0; }),
                   mc_rtc::gui::NumberInput(
                       "Duration", [this]() { return duration_; }, [this](double duration) { duration_ = duration; }));
  }

  void update();

  inline void clear()
  {
    poses_.clear();
    velocities_ = std::nullopt;
    forces_ = std::nullopt;
  }

  /**
   * @brief In case no velocity is provided, computes it by derivating the
   * provided pose
   */
  void computeVelocity();

  inline const std::vector<sva::PTransformd> & poses() const noexcept
  {
    return poses_;
  }

  inline void name(const std::string & name)
  {
    name_ = name;
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  inline void robot(const std::string & robot)
  {
    robot_ = robot;
  }

  inline const std::string & robot() const noexcept
  {
    return robot_;
  }

  inline void robotFrame(const std::string & frame)
  {
    robotFrame_ = frame;
  }

  inline const std::string & robotFrame() const noexcept
  {
    return robotFrame_;
  }

private:
  std::string name_{"Unnamed Trajectory"};
  std::string robot_{""};
  std::string robotFrame_{""};
  sva::PTransformd refAxis_{sva::PTransformd::Identity()}; ///< Reference axis expressed w.r.t refFrame_
  double duration_ = 5; ///< Duration of the trajectory
  double dt_ = 0;
  std::vector<sva::PTransformd> poses_; ///< Desired pose defined w.r.t refAxis_
  std::optional<std::vector<sva::MotionVecd>> velocities_; ///< Desired velocity defined w.r.t
                                                           ///  refAxis_
  std::optional<std::vector<sva::ForceVecd>>
      forces_; ///< Desired force defined w.r.t the pose frame along the trajectory
};
