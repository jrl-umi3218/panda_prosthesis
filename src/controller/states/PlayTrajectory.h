#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/ImpedanceTask.h>
#include "../include/trajectory.h"

struct TrajectoryPlayer
{
  TrajectoryPlayer(mc_solver::QPSolver & solver, const Trajectory & traj) : solver_(solver), trajectory_(traj)
  {
    task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(traj.frame());
    task_->stiffness(1000);
    task_->gains().wrench().linear({0,0,0});
    task_->reset();
    solver.addTask(task_);
    mc_rtc::log::info("Add impedance task for robot {}", traj.frame().robot().name());
  }

  ~TrajectoryPlayer()
  {
    mc_rtc::log::error("TrajectoryPlayer::~TrajectoryPlayer for frame {}", task_->frame().name());
    solver_.removeTask(task_);
  }

  void update(double dt)
  {
    if(t_ < trajectory_.duration())
    {
      const auto & pose = trajectory_.worldPose(t_);
      const auto & velocity = trajectory_.worldVelocity(t_);
      const auto & wrench = trajectory_.worldWrench(t_);

      task_->targetPose(pose);
      task_->targetVel(velocity);
      task_->targetWrenchW(wrench);
      mc_rtc::log::info("Update for frame {}\nRPY : {}\nVel : {}\nForce: {}", trajectory_.frame().name(),
                        pose.translation().transpose(), velocity.angular().transpose(), wrench.force().transpose());

      t_ = ++n_ * dt;
    }
    else
    {
      task_->targetVel(sva::MotionVecd::Zero());
      task_->targetWrench(sva::ForceVecd::Zero());
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
