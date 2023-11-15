#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_rtc/log/Logger.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <PandaProsthesisImpedanceTask.h> // custom impedance task
#include <TrajectoryPlayer.h>

TrajectoryPlayer::TrajectoryPlayer(mc_control::fsm::Controller & ctl,
                                   const Trajectory & traj,
                                   const mc_rtc::Configuration & config)
: ctl_(ctl), trajectory_(traj)
{
  auto & solver = ctl_.solver();
  const auto & frame = traj.frame();
  // task_ = std::make_shared<mc_tasks::force::PandaProsthesisImpedanceTask>(*traj.frame(), *traj.refAxisFrame(),
  //                                                                         *traj.frame());
  task_ = std::make_shared<mc_tasks::force::PandaProsthesisImpedanceTask>(*traj.frame(), *traj.refAxisFrame(),
                                                                          traj.frame()->robot().frame("panda_link7"));
  task_->reset();
  if(config.has("impedanceTask"))
  {
    task_->load(solver, config("impedanceTask"));
  }
  solver.addTask(task_);

  pause_ = !config("autoplay", true);
  config("applyForceWhenPaused", applyForceWhenPaused_);
  config("trackForce", trackForce_);
  config("manualForce", manualForce_);
  config("manualWrench", manualWrench_);

  wrenchGains_ = task_->gains().wrench().vector();
  mc_rtc::log::info("[TrajectoryPlayer::{}] Created TrajectoryPlayer for trajectory {}", traj.name(), traj.name());
  mc_rtc::log::info(
      "[TrajectoryPlayer::{}] Controlling frame {} of robot {} with an impedance task configured as follows:\n{}",
      frame->name(), frame->name(), frame->robot().name(), config.dump(true));
}

void TrajectoryPlayer::addToLogger(mc_rtc::Logger & logger)
{
  const auto & frame = trajectory_.frame();
  logger.addLogEntry("trackForce", [this]() { return trackForce_; });
  logger.addLogEntry("target_wrench", [this]() -> const sva::ForceVecd & { return task_->targetWrench(); });
  logger.addLogEntry("measured_wrench", [this]() -> const sva::ForceVecd & { return task_->filteredMeasuredWrench(); });
  logger.addLogEntry(fmt::format("mesured_{}", trajectory_.refAxisFrame()->forceSensor().name()),
                     [this]() { return trajectory_.refAxisFrame()->wrench(); });
  logger.addLogEntry(fmt::format("transform_control_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()), [this]()
                     { return trajectory_.frame()->position() * trajectory_.refAxisFrame()->position().inv(); });
  logger.addLogEntry(fmt::format("transform_real_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()),
                     [this]()
                     {
                       auto & frame = *trajectory_.frame();
                       return ctl_.realRobot(frame.robot().name()).frame(frame.name()).position()
                              * trajectory_.refAxisFrame()->position().inv();
                     });
  auto sensors = std::vector<std::string>{"Sensor0", "Sensor1"};
  for(const auto & sensorName : sensors)
  {
    logger.addLogEntry("PhidgetPressureSensor_" + sensorName + "_pressure",
                       [this, sensorName]()
                       {
                         if(ctl_.datastore().has("PhidgetPressureSensor::" + sensorName + "::pressure"))
                         {
                           return ctl_.datastore().call<double>("PhidgetPressureSensor::" + sensorName + "::pressure");
                         }
                         else
                         {
                           return 0.;
                         }
                       });
  }
}

TrajectoryPlayer::~TrajectoryPlayer()
{
  mc_rtc::log::error("TrajectoryPlayer::~TrajectoryPlayer for frame {}", task_->frame().name());
  ctl_.solver().removeTask(task_);
  if(log_)
  {
    log_->flush();
  }
}

void TrajectoryPlayer::addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back(trajectory_.name());
  gui.addElement(this, category, mc_rtc::gui::Input("Pause", pause_), mc_rtc::gui::Input("Track Force", trackForce_),
                 mc_rtc::gui::Input("Apply force when paused", applyForceWhenPaused_),
                 mc_rtc::gui::Input("Force Scaling (ratio)", forceScaling_));
  gui.addElement(
      this, category,
      mc_rtc::gui::Form(
          "Start Logging",
          [this](const mc_rtc::Configuration & data)
          {
            auto filename = fmt::format("{}-LP_{:.3f}-RP_{:.3f}-{}", data("Patient Name"),
                                        static_cast<double>(data("Left Pressure")),
                                        static_cast<double>(data("Right Pressure")), static_cast<int>(data("Trial")));
            log_ = std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED,
                                                    static_cast<std::string>(ctl_.config()("results_dir")),
                                                    "trajectory-player");
            log_->start(filename, ctl_.timeStep, true);
            addToLogger(*log_);
            logging_ = true;
          },
          mc_rtc::gui::FormStringInput("Patient Name", true, "Toto"),
          mc_rtc::gui::FormNumberInput("Left Pressure", true, 0),
          mc_rtc::gui::FormNumberInput("Right Pressure", true, 0), mc_rtc::gui::FormIntegerInput("Trial", true, 0)));
  gui.addElement(this, category, mc_rtc::gui::Button("Stop logging", [this]() { logging_ = false; }));

  auto playingStatusCat = category;
  playingStatusCat.push_back("Playing Status");
  gui.addElement(this, category, mc_rtc::gui::NumberSlider("t: ", t_, 0, trajectory_.duration()),
                 mc_rtc::gui::ArrayLabel("Target Translation (World): ", task_->targetPose().translation()),
                 mc_rtc::gui::RPYLabel("Target Rotation (World): ", task_->targetPose().rotation()));
  category.push_back("Manual");
  gui.addElement(this, category, mc_rtc::gui::Input("Manual Target Force", manualForce_),
                 mc_rtc::gui::Input("Manual Target Wrench (Tibia frame)", manualWrench_),
                 mc_rtc::gui::Input("Wrench Gains", wrenchGains_));
  gui.addElement(this, category,
                 mc_rtc::gui::Force(
                     "Target Force Femur", [this]() -> sva::ForceVecd { return task_->targetWrench(); },
                     [this]() -> sva::PTransformd { return task_->frame().position(); }));
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
      task_->gains().wrench() = wrenchGains_;
      if(manualForce_)
      { // manual force
        auto refAxis = trajectory_.refAxisFrame()->position();
        auto manualWrenchW = refAxis.inv().dualMul(manualWrench_);
        task_->targetWrenchW(manualWrenchW);
      }
      else
      { // force from trajectory
        const auto & wrench = trajectory_.worldWrench(t_);
        task_->targetWrenchW(forceScaling_ * wrench);
      }
    }
    else
    {
      task_->targetWrench(sva::ForceVecd::Zero());
      task_->gains().wrench() = sva::ImpedanceVecd::Zero();
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

    if(t_ >= trajectory_.duration()) // trajectory is finished
    {
      logging_ = false;
    }
  }

  if(applyForceWhenPaused_)
  {
    trackForce();
  }
  if(log_ && logging_)
  {
    log_->log();
  }
}
