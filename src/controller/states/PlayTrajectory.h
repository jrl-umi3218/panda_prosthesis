#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/ImpedanceTask.h>
#include "../include/trajectory.h"

struct TrajectoryPlayer
{
  TrajectoryPlayer(mc_solver::QPSolver & solver, const Trajectory & traj) : solver_(solver), trajectory_(traj)
  {
    // XXX fixme allow frames in ImpedanceTask
    mc_rtc::log::info("Make task for robot {}", traj.frame().robot().name());
    task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(traj.frame());
    task_->reset();
    solver.addTask(task_);
  }

  ~TrajectoryPlayer()
  {
    mc_rtc::log::error("TrajectoryPlayer::~TrajectoryPlayer for frame {}", task_->frame().name());
    solver_.removeTask(task_);
  }

  void update(double dt)
  {
    const auto & pose = trajectory_.pose(t_);
    const auto & velocity = trajectory_.velocity(t_);

    mc_rtc::log::info("Update for frame {}", trajectory_.frame().name());
    // XXX define w.r.t the right frame?
    task_->targetPose(pose);
    // task_->targetVel(velocity);

    if(t_ < trajectory_.duration())
    {
      t_ = ++n_ * dt;
    }
  }

  inline bool finished() const noexcept
  {
    return t_ >= trajectory_.duration();
  }

 protected:
  mc_solver::QPSolver & solver_;
  const Trajectory & trajectory_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> task_;
  unsigned n_ = 0;
  double t_ = 0;
  unsigned iter_ = 0;
};

struct PlayTrajectory : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;
 protected:
  inline bool finished() const noexcept
  {
    for(const auto & player : trajPlayers_)
    {
      if(!player->finished())
      {
        return false;
      }
    }
    return true;
  }
  std::vector<std::shared_ptr<TrajectoryPlayer>> trajPlayers_;
};
