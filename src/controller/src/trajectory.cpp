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
    poses_.emplace_back(mc_rbdyn::rpyToMat(rotation).inverse(), translation);
  }
}

void Trajectory::update()
{
  if(poses_.empty())
  {
    mc_rtc::log::error_and_throw("[Trajectory::update] Must have at least one pose, none provided");
  }
  dt_ = duration_ / poses_.size();
  computeVelocity();
}

void Trajectory::computeVelocity()
{
  if(velocities_)
  {
    velocities_->clear();
  }
  else
  {
    velocities_ = {};
  }
  velocities_->push_back(sva::MotionVecd::Zero());
  for(int i = 0; i < poses_.size() - 1; ++i)
  {
    const auto & X_a = poses_[i];
    const auto & X_b = poses_[i + 1];
    auto X_a_b = X_b * X_a.inv();
    velocities_->push_back(sva::transformVelocity(X_a_b) * dt_);
  }
}
