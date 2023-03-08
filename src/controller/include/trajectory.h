#pragma once
#include <mc_rtc/gui.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <optional>

/**
 * Functor to interpolate values in-between two poses expressed as sva::PTransformd
 */
struct PoseInterpolation
{
  sva::PTransformd operator()(const sva::PTransformd & p1, const sva::PTransformd & p2, double t) const
  {
    return sva::interpolate(p1, p2, t);
  }
};

struct Trajectory
{
  Trajectory(const std::string & name, const mc_rbdyn::RobotFrame & frame, const mc_rbdyn::RobotFrame & refAxisFrame)
  : name_(name), frame_(frame), refAxisFrame_(refAxisFrame)
  {
    // By default rotate around the robot frame where the trajectory was created
    refAxis_ = refAxisFrame_->position();
    refForceAxis_ = refAxisFrame_->position();
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
                        const std::string & fz);

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
  {
    category.push_back(name_);
    using namespace mc_rtc::gui;
    gui.addElement(
        this, category, mc_rtc::gui::Label("Robot", [this]() { return frame_->robot().name(); }),
        mc_rtc::gui::Label("Frame", [this]() { return frame_->name(); }),
        mc_rtc::gui::Label("Poses", [this]() { return poses_.size(); }),
        mc_rtc::gui::Label("Velocities", [this]() { return velocities_.size(); }),
        mc_rtc::gui::Label("Forces", [this]() { return forces_ ? forces_->size() : 0; }),
        mc_rtc::gui::NumberInput(
            "Duration", [this]() { return duration_; }, [this](double duration) { this->duration(duration); }));
  }

  /**
   * Add an initial point at t=0 corresponding to the initial world pose and
   * velocity of the control frame on the robot. Shift the whole trajectory by
   * interpolationTime.
   *
   * This may be used to prevent the robot from jumping to the initial position
   * of the trajectory
   */
  void interpolateInitialPose(const sva::PTransformd & initialRobotFrame,
                              const sva::MotionVecd & initialRobotFrameVelocity = sva::MotionVecd::Zero(),
                              double interpolationTime = 2.0)
  {
    duration_ += interpolationTime;
    {
      auto values = poseInterpolation_.values();
      auto newValues = PoseInterpolator::TimedValueVector{};
      newValues.reserve(values.size() + 1);
      auto X_refAxis_initalRobotFrame = initialRobotFrame * refAxis_.inv();
      newValues.emplace_back(0.0, X_refAxis_initalRobotFrame);
      for(const auto & [t, pose] : values)
      {
        newValues.emplace_back(t + interpolationTime, pose);
      }
      poseInterpolation_.values(newValues);
    }
    for(const auto & [t, pose] : poseInterpolation_.values())
    {
      mc_rtc::log::info("t: {}, pose: {}", t, pose.translation().transpose());
    }
    {
      auto values = velocityInterpolation_.values();
      auto newValues = VelocityInterpolator::TimedValueVector{};
      newValues.reserve(values.size() + 1);
      newValues.emplace_back(0.0, refAxis_.inv() * initialRobotFrameVelocity);
      for(const auto & [t, vel] : values)
      {
        newValues.emplace_back(t + interpolationTime, vel);
      }
      velocityInterpolation_.values(newValues);
    }
  }

  void update();

  void reverse();

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

  /*
   * Returns the interpolated world pose at time t
   */
  inline const sva::PTransformd worldPose(double t)
  {
    return poseInterpolation_.compute(t) * refAxis_;
  }

  inline const std::vector<sva::PTransformd> & poses() const noexcept
  {
    return poses_;
  }

  /*
   * Returns the interpolated world velocity at time t
   */
  inline const sva::MotionVecd worldVelocity(double t)
  {
    return refAxis_ * velocityInterpolation_.compute(t);
  }

  inline const std::vector<sva::MotionVecd> & velocities() const noexcept
  {
    return velocities_;
  }

  /**
   * Returns the discrete desired world wrench for time t
   */
  inline const sva::ForceVecd worldWrench(double t) const
  {
    if(forces_)
    {
      return refForceAxis_.dualMul((*forces_)[indexFromTime(t)]);
    }
    return sva::ForceVecd::Zero();
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
    return std::min(static_cast<unsigned>(floor(t / dt_)), static_cast<unsigned>(poses_.size() - 1));
  }

private:
  bool needUpdate_ = true;
  std::string name_{};
  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_rbdyn::ConstRobotFramePtr refAxisFrame_;
  sva::PTransformd refAxis_{sva::PTransformd::Identity()}; ///< Reference axis in world frame
  sva::PTransformd refForceAxis_{sva::PTransformd::Identity()}; ///< Reference force axis in world frame
  double duration_ = 30; ///< Duration of the trajectory
  double dt_ = 0;
  std::vector<sva::PTransformd> poses_; ///< Desired pose defined w.r.t refAxis_
  std::vector<sva::MotionVecd> velocities_; ///< Desired velocity defined w.r.t refAxis_
  std::optional<std::vector<sva::ForceVecd>> forces_; ///< Desired force defined w.r.t refForceAxis_
  using PoseInterpolator = mc_trajectory::SequenceInterpolator<sva::PTransformd, PoseInterpolation>;
  using VelocityInterpolator = mc_trajectory::SequenceInterpolator<sva::MotionVecd>;
  PoseInterpolator poseInterpolation_;
  VelocityInterpolator velocityInterpolation_;
};
