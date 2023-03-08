#include "trajectory.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include "../3rd-party/csv.h"

void Trajectory::loadPoseFromCSV(const std::string & csv,
                                 const std::string & tangage,
                                 const std::string & roulis,
                                 const std::string & lacet,
                                 const std::string & tx,
                                 const std::string & ty,
                                 const std::string & tz)
{
  io::CSVReader<6> in(csv);
  Eigen::Vector3d rotation, translation;
  in.read_header(io::ignore_extra_column, tangage, roulis, lacet, tx, ty, tz);
  while(in.read_row(rotation[0], rotation[1], rotation[2], translation[0], translation[1], translation[2]))
  {
    poses_.emplace_back(mc_rbdyn::rpyToMat(mc_rtc::constants::PI / 180. * rotation), translation / 1000.);
  }
}

void Trajectory::loadForceFromCSV(const std::string & csv,
                                  const std::string & cx,
                                  const std::string & cy,
                                  const std::string & cz,
                                  const std::string & fx,
                                  const std::string & fy,
                                  const std::string & fz)
{
  io::CSVReader<6> in(csv);
  sva::ForceVecd wrench = sva::ForceVecd::Zero();
  forces_ = std::vector<sva::ForceVecd>{};
  in.read_header(io::ignore_extra_column, cx, cy, cz, fx, fy, fz);
  while(in.read_row(wrench.couple().x(), wrench.couple().y(), wrench.couple().z(), wrench.force().x(),
                    wrench.force().y(), wrench.force().z()))
  {
    forces_->push_back(wrench);
  }
}

void Trajectory::update()
{
  if(needUpdate_)
  {
    if(poses_.empty())
    {
      mc_rtc::log::error_and_throw("[Trajectory::update] Must have at least one pose, none provided");
    }
    dt_ = duration_ / poses_.size();
    PoseInterpolator::TimedValueVector posesInterp;
    for(int i = 0; i < poses_.size(); ++i)
    {
      const auto & pose = poses_[i];
      posesInterp.emplace_back(dt_ * i, pose);
    }
    poseInterpolation_.values(posesInterp);
    computeVelocity();
    VelocityInterpolator::TimedValueVector velInterp;
    for(int i = 0; i < velocities_.size(); ++i)
    {
      const auto & vel = velocities_[i];
      velInterp.emplace_back(dt_ * i, vel);
    }
    velocityInterpolation_.values(velInterp);
    needUpdate_ = false;
  }
}

void Trajectory::reverse()
{
  auto revPoses = std::vector<sva::PTransformd>{};
  revPoses.reserve(poses_.size());
  for(auto it = poses_.rbegin(); it != poses_.rend(); ++it)
  {
    revPoses.push_back(*it);
  }
  poses_ = revPoses;
  if(forces_)
  {
    auto revForces = std::vector<sva::ForceVecd>{};
    revForces.reserve(forces_->size());
    for(auto it = forces_->rbegin(); it != forces_->rend(); ++it)
    {
      revForces.push_back(*it);
    }
    forces_ = revForces;
  }
  needUpdate_ = true;
  update();
}

void Trajectory::computeVelocity()
{
  if(velocities_.size())
  {
    velocities_.clear();
  }
  velocities_.emplace_back(sva::MotionVecd::Zero());
  for(int i = 0; i < poses_.size() - 1; ++i)
  {
    const auto & X_a = poses_[i];
    const auto & X_b = poses_[i + 1];
    auto X_a_b = X_b * X_a.inv();
    velocities_.push_back(sva::transformVelocity(X_a_b) * dt_);
  }
}
