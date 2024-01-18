#pragma once
#include <mc_control/fsm/Controller.h>
#include <mc_solver/QPSolver.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Trajectory.h>

namespace mc_tasks
{
namespace force
{
struct PandaProsthesisImpedanceTask;
}
} // namespace mc_tasks

struct TrajectoryPlayer
{
  TrajectoryPlayer(mc_control::fsm::Controller & ctl,
                   const Trajectory & traj,
                   const mc_rtc::Configuration & config = mc_rtc::Configuration{});

  void addToGUI(mc_control::fsm::Controller & ctl, std::vector<std::string> category);
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category);

  ~TrajectoryPlayer();

  void update(double dt);

  inline void pause(bool pause)
  {
    pause_ = pause;
  }

  inline bool finished() const noexcept
  {
    return t_ >= trajectory_.duration();
  }

  inline const Trajectory & trajectory() const noexcept
  {
    return trajectory_;
  }

  void addToLogger(mc_control::fsm::Controller & ctl, mc_rtc::Logger & logger);

protected:
  mc_control::fsm::Controller & ctl_;
  Trajectory trajectory_;
  std::shared_ptr<mc_tasks::force::PandaProsthesisImpedanceTask> task_;
  sva::ImpedanceVecd wrenchGains_ = sva::ImpedanceVecd::Zero();
  unsigned n_ = 0;
  double t_ = 0;
  unsigned iter_ = 0;
  bool pause_ = false;
  bool trackForce_ = true; /// Whether to track the desired forces
  bool applyForceWhenPaused_ = true;
  bool manualForce_ = false; /// Whether to apply a manually specified force
                             /// instead of the one from the trajectory
  double forceScaling_ = 1;
  sva::ForceVecd manualWrench_ = sva::ForceVecd::Zero(); /// Manual wrench (only
                                                         /// used if manualForce_ = true
  bool logging_ = false;
  std::shared_ptr<mc_rtc::Logger> log_ = nullptr;
};
