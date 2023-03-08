#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/ImpedanceTask.h>
#include "../include/trajectory.h"

struct TrajectoryPlayer
{
  TrajectoryPlayer(mc_solver::QPSolver & solver,
                   const Trajectory & traj,
                   const mc_rtc::Configuration & config = mc_rtc::Configuration{})
  : solver_(solver), trajectory_(traj)
  {
    task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(traj.frame());
    if(config.has("impedanceTask"))
    {
      task_->load(solver, config("impedanceTask"));
    }
    pause_ = !config("autoplay", true);
    config("applyForceWhenPaused", applyForceWhenPaused_);
    config("trackForce", trackForce_);
    config("manualForce", manualForce_);
    config("manualWrench", manualWrench_);

    task_->reset();
    solver.addTask(task_);
    mc_rtc::log::info("[TrajectoryPlayer::{}] Created TrajectoryPlayer for trajectory {}", traj.name(), traj.name());
    mc_rtc::log::info(
        "[TrajectoryPlayer::{}] Controlling frame {} of robot {} with an impedance task configured as follows:\n{}",
        traj.name(), traj.frame().name(), traj.frame().robot().name(), config.dump(true));
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui)
  {
    gui.addElement(this, {"TrajectoryPlayer", trajectory_.name()}, mc_rtc::gui::Input("Pause", pause_),
                   mc_rtc::gui::Input("Track Force", trackForce_),
                   mc_rtc::gui::Input("Apply force when paused", applyForceWhenPaused_));
    gui.addElement(this, {"TrajectoryPlayer", trajectory_.name(), "Manual"},
                   mc_rtc::gui::Input("Manual Target Force", manualForce_),
                   mc_rtc::gui::Input("Manual Target Wrench", manualWrench_));
  }

  void removeFromGUI(mc_rtc::gui::StateBuilder & gui)
  {
    gui.removeElements(this);
  }

  ~TrajectoryPlayer()
  {
    mc_rtc::log::error("TrajectoryPlayer::~TrajectoryPlayer for frame {}", task_->frame().name());
    solver_.removeTask(task_);
  }

  void update(double dt)
  {
    auto trackForce = [this]() {
      // Track force from the trajectory
      if(trackForce_)
      {
        if(manualForce_)
        { // manual force
          task_->targetWrenchW(manualWrench_);
        }
        else
        { // force from trajectory
          const auto & wrench = trajectory_.worldWrench(t_);
          task_->targetWrenchW(wrench);
        }
      }
      else
      {
        task_->targetWrench(sva::ForceVecd::Zero());
      }
    };

    // Apply a manual force even when paused
    if(t_ < trajectory_.duration() && !pause_)
    {
      const auto pose = trajectory_.worldPose(t_);
      const auto velocity = trajectory_.worldVelocity(t_);

      task_->targetPose(pose);
      task_->targetVel(velocity);
      trackForce();

      t_ = ++n_ * dt;
    }
    else
    { // paused or trajectory is finished
      task_->targetVel(sva::MotionVecd::Zero());
      task_->targetWrench(sva::ForceVecd::Zero());
    }

    if(applyForceWhenPaused_)
    {
      trackForce();
    }
  }

  void pause(bool pause)
  {
    pause_ = pause;
  }

  inline bool finished() const noexcept
  {
    return t_ >= trajectory_.duration();
  }

protected:
  mc_solver::QPSolver & solver_;
  Trajectory trajectory_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> task_;
  unsigned n_ = 0;
  double t_ = 0;
  unsigned iter_ = 0;
  bool pause_ = false;
  bool trackForce_ = true; /// Whether to track the desired forces
  bool applyForceWhenPaused_ = true;
  bool manualForce_ = false; /// Whether to apply a manually specified force
                             /// instead of the one from the trajectory
  sva::ForceVecd manualWrench_ = sva::ForceVecd::Zero(); /// Manual wrench (only
                                                         /// used if manualForce_ = true
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
