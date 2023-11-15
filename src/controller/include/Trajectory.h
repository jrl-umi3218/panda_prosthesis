#pragma once
#include <mc_rtc/gui.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <optional>

/**
 * @param flexionAngle [rad]
 * Expressed in tibia frame
 */
inline sva::ForceVecd computeNormalForce(double flexionAngle)
{
  sva::ForceVecd force = sva::ForceVecd::Zero();
  force.force().z() = -47.331 * std::pow(flexionAngle, 6) + 150.97 * std::pow(flexionAngle, 5)
                      - 168.73 * std::pow(flexionAngle, 4) + 81.151 * std::pow(flexionAngle, 3)
                      - 23.117 * std::pow(flexionAngle, 2) + 19.756 * flexionAngle + 5.3681;
  return force;
}

/**
 * Functor to interpolate values in-between two poses expressed as sva::PTransformd
 */
struct PoseInterpolation
{
  // FIXME using RPY for now as both sva::interpolate and Quaternion slerp fail
  sva::PTransformd operator()(const sva::PTransformd & p1, const sva::PTransformd & p2, double t) const
  {
    return sva::interpolate(p1, p2, t);
  }
};

struct PoseInterpolationSlerp
{
  sva::PTransformd operator()(const sva::PTransformd & p1, const sva::PTransformd & p2, double t) const
  {
    Eigen::Quaterniond r1{p1.rotation()};
    Eigen::Quaterniond r2{p2.rotation()};
    Eigen::Matrix3d rslerp = r1.slerp(t, r2).toRotationMatrix().eval();
    auto t1 = p1.translation();
    auto t2 = p2.translation();
    return {rslerp, t1 + (t2 - t1) * t};
  }
};

/*
 * Simplistic interpolation:
 * - Rotations are interpolated in RPY space. It can fail when crossing RPY discontinuities
 * - Translations are a simple linear interpolation
 *
 * Prefer using a more eleborate method, this is only intended for quick checks
 */
struct RPYPoseInterpolation
{
  sva::PTransformd operator()(const sva::PTransformd & p1, const sva::PTransformd & p2, double t) const
  {
    // Interpolate using RPY
    auto rpy1 = mc_rbdyn::rpyFromMat(p1.rotation());
    auto rpy2 = mc_rbdyn::rpyFromMat(p2.rotation());
    auto t1 = p1.translation();
    auto t2 = p2.translation();

    return {mc_rbdyn::rpyToMat(rpy1 + (rpy2 - rpy1) * t), t1 + (t2 - t1) * t};
    /* mc_rtc::log::info("Interpolation between\n\t{}\n\t{}\n\t{}\n\t{},\n\tt={}", */
    /*     mc_rbdyn::rpyFromMat(p1.rotation()).transpose() * 180/3.14, */
    /*     mc_rbdyn::rpyFromMat(p2.rotation()).transpose() * 180/3.14, */
    /*     mc_rbdyn::rpyFromMat(interpolated.rotation()).transpose() * 180/3.14, */
    /*     interpolated.translation().transpose(), */
    /*     t */
    /*     ); */
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

  void loadPoses(const mc_rtc::Configuration & config);

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
                       "Flexion Angular Velocity [deg/s]", [this]() { return duration_; },
                       [this](double duration) { this->duration(duration); }));
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
      poseInterpolation_.clear();
      poseInterpolation_.values(newValues);
    }
    for(const auto & [t, pose] : poseInterpolation_.values())
    {
      mc_rtc::log::info("t: {}, translation: {}, rotation (deg): {}", t, pose.translation().transpose(),
                        mc_rbdyn::rpyFromMat(pose.rotation()).transpose() * 180 / mc_rtc::constants::PI);
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
   * Desired wrench expressed in refForceAxis_ frame
   */
  inline const sva::ForceVecd wrench(double t)
  {
    if(forces_)
    {
      return (*forces_)[indexFromTime(t)];
    }
    else
    {
      // compute force
      auto pose = poseInterpolation_.compute(t);
      double flexionAngle = mc_rbdyn::rpyFromMat(pose.rotation()).x();
      return computeNormalForce(flexionAngle);
    }
    return sva::ForceVecd::Zero();
  }

  /**
   * Returns the discrete desired world wrench for time t
   */
  inline const sva::ForceVecd worldWrench(double t)
  {
    return refForceAxis_.dualMul(wrench(t));
  }

  inline void name(const std::string & name)
  {
    name_ = name;
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  const mc_rbdyn::ConstRobotFramePtr frame() const noexcept
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

  inline mc_rbdyn::ConstRobotFramePtr refAxisFrame() const noexcept
  {
    return refAxisFrame_;
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
  double flexionAngularVelocity_ =
      0.0407; // previous value : 0.122 Desired angular velocity for the flexion angle (constant) [rad/s]
  double duration_ = 0; ///< Duration of the trajectory
  double dt_ = 0;
  std::vector<sva::PTransformd> poses_; ///< Desired pose defined w.r.t refAxis_
  std::vector<sva::MotionVecd> velocities_; ///< Desired velocity defined w.r.t refAxis_
  std::optional<std::vector<sva::ForceVecd>> forces_; ///< Desired force defined w.r.t refForceAxis_
  // FIXME We should not be using RPYPoseInterpolation but for now the
  // interpolation using sva::interpolate fails
  using PoseInterpolator = mc_trajectory::SequenceInterpolator<sva::PTransformd, RPYPoseInterpolation>;
  using VelocityInterpolator = mc_trajectory::SequenceInterpolator<sva::MotionVecd>;
  PoseInterpolator poseInterpolation_;
  VelocityInterpolator velocityInterpolation_;
};
