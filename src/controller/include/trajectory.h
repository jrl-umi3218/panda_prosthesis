#pragma once
#include <mc_rtc/gui.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <optional>

struct Trajectory
{
  Trajectory(const std::string & name, const mc_rbdyn::RobotFrame & frame) : name_(name), frame_(frame)
  {
  }

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
    gui.addElement(this, category, mc_rtc::gui::Label("Robot", [this]() { return frame_->robot().name(); }),
                   mc_rtc::gui::Label("Frame", [this]() { return frame_->name(); }),
                   mc_rtc::gui::Label("Poses", [this]() { return poses_.size(); }),
                   mc_rtc::gui::Label("Velocities", [this]() { return velocities_.size(); }),
                   mc_rtc::gui::Label("Forces", [this]() { return forces_ ? forces_->size() : 0; }),
                   mc_rtc::gui::NumberInput(
                       "Duration", [this]() { return duration_; }, [this](double duration) { this->duration(duration); }));
  }

  void update();

  inline void clear()
  {
    dt_ = 0;
    poses_.clear();
    velocities_.clear();
    forces_ = std::nullopt;
  }

  /**
   * @brief In case no velocity is provided, computes it by derivating the
   * provided pose
   */
  void computeVelocity();

  inline const sva::PTransformd & pose(double t) const
  {
    mc_rtc::log::info("pose time: {}, index: {}", t, indexFromTime(t));
    return poses_[indexFromTime(t)];
  }

  inline const std::vector<sva::PTransformd> & poses() const noexcept
  {
    return poses_;
  }

  inline const sva::MotionVecd & velocity(double t) const
  {
    return velocities_[indexFromTime(t)];
  }

  inline const std::vector<sva::MotionVecd> & velocities() const noexcept
  {
    return velocities_;
  }

  inline void name(const std::string & name)
  {
    name_ = name;
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return *frame_;
  }

  inline void duration(double duration)
  {
    duration_ = duration;
    needUpdate_ = true;
  }

  inline void rate(double rate)
  {
    dt_ = rate;
    duration(dt_ * poses_.size());
  }

  inline double duration() const noexcept
  {
    return duration_;
  }

 protected:
  unsigned indexFromTime(double t) const noexcept
  {
    if(poses_.empty()) return 0;
    return std::min(static_cast<unsigned>(floor(t / dt_)), static_cast<unsigned>(poses_.size() - 1)) ;
  }

private:
  bool needUpdate_ = true;
  std::string name_{};
  mc_rbdyn::ConstRobotFramePtr frame_;
  // std::string robot_{""};
  // std::string robotFrame_{""};
  sva::PTransformd refAxis_{sva::PTransformd::Identity()}; ///< Reference axis expressed w.r.t refFrame_
  double duration_ = 5; ///< Duration of the trajectory
  double dt_ = 0;
  std::vector<sva::PTransformd> poses_; ///< Desired pose defined w.r.t refAxis_
  std::vector<sva::MotionVecd> velocities_; ///< Desired velocity defined w.r.t refAxis_
  std::optional<std::vector<sva::ForceVecd>>
      forces_; ///< Desired force defined w.r.t the pose frame along the trajectory
};
