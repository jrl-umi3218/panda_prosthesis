#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_tasks/ImpedanceTask.h>
#include <TrajectoryPlayer.h>

TrajectoryPlayer::TrajectoryPlayer(mc_solver::QPSolver & solver,
                                   const Trajectory & traj,
                                   const mc_rtc::Configuration & config)
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

TrajectoryPlayer::~TrajectoryPlayer()
{
  mc_rtc::log::error("TrajectoryPlayer::~TrajectoryPlayer for frame {}", task_->frame().name());
  solver_.removeTask(task_);
}

void TrajectoryPlayer::addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back(trajectory_.name());
  gui.addElement(this, category, mc_rtc::gui::Input("Pause", pause_), mc_rtc::gui::Input("Track Force", trackForce_),
                 mc_rtc::gui::Input("Apply force when paused", applyForceWhenPaused_));
  auto playingStatusCat = category;
  playingStatusCat.push_back("Playing Status");
  gui.addElement(this, category, mc_rtc::gui::NumberSlider("t: ", t_, 0, trajectory_.duration()),
                 mc_rtc::gui::ArrayLabel("Target Translation (World): ", task_->targetPose().translation()),
                 mc_rtc::gui::RPYLabel("Target Rotation (World): ", task_->targetPose().rotation()));
  category.push_back("Manual");
  gui.addElement(this, category, mc_rtc::gui::Input("Manual Target Force", manualForce_),
                 mc_rtc::gui::Input("Manual Target Wrench (Tibia frame)", manualWrench_));
                 gui.addElement(this, category, mc_rtc::gui::Force("Target Force Femur", 
                 [this]() -> sva::ForceVecd
                  {
                    return task_->targetWrench();
                  },
                  [this]() -> sva::PTransformd
                  {
                    task_->frame().position();
                  }                  
                 ));
}

void TrajectoryPlayer::removeFromGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back(trajectory_.name());
  gui.removeCategory(category);
}

void TrajectoryPlayer::update(double dt)
{
  auto trackForce = [this]()
  {
    // Track force from the trajectory
    if(trackForce_)
    {
      if(manualForce_)
      { // manual force
        auto refAxis = trajectory_.refAxisFrame()->position();
        auto manualWrenchW = refAxis.inv().dualMul(manualWrench_);
        task_->targetWrenchW(manualWrenchW);
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
    /* mc_rtc::log::info("Computed pose for t={} is {}", t_, mc_rbdyn::rpyFromMat(pose.rotation()).transpose() * 180 /
     * mc_rtc::constants::PI); */
    const auto velocity = trajectory_.worldVelocity(t_);

    task_->targetPose(pose);
    task_->targetVel(velocity);
    trackForce();

    t_ = ++n_ * dt;
  }
  else
  { // paused or trajectory is finished
    /* mc_rtc::log::critical("Trajectory is finished"); */
    task_->targetWrench(sva::ForceVecd::Zero());
    task_->targetVel(sva::MotionVecd::Zero());
  }

  if(applyForceWhenPaused_)
  {
    trackForce();
  }
}
