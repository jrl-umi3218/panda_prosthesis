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
  // use ATI force sensor
  task_ = std::make_shared<mc_tasks::force::PandaProsthesisImpedanceTask>(*traj.frame(), *traj.refAxisFrame(),
                                                                          *traj.frame());
  // Use robot's LeftHandForceSensor
  /* task_ = std::make_shared<mc_tasks::force::PandaProsthesisImpedanceTask>(*traj.frame(), *traj.refAxisFrame(), */
  /*                                                                         traj.frame()->robot().frame("panda_link7"));
   */
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

void TrajectoryPlayer::addToLogger(mc_control::fsm::Controller & ctl, mc_rtc::Logger & logger)
{
  const auto & frame = trajectory_.frame();
  const auto & refAxisFrame = trajectory_.refAxisFrame();
  logger.addLogEntry("trackForce", [this]() { return trackForce_; });

  logger.addLogEntry(fmt::format("control_target_wrench-{}", refAxisFrame->name()),
                     [this]() -> const sva::ForceVecd & { return task_->targetWrench(); });
  logger.addLogEntry(fmt::format("control_measured_wrench-{}", refAxisFrame->name()),
                     [this]() -> const sva::ForceVecd & { return task_->filteredMeasuredWrench(); });
  logger.addLogEntry(fmt::format("control_measured_wrench_norm-{}", refAxisFrame->name()),
                     [this]() { return task_->filteredMeasuredWrench().force().norm(); });

  auto log_sensor = [&logger](const mc_rbdyn::RobotFrame & sensorFrame, const mc_rbdyn::RobotFrame & refFrame)
  {
    auto log_prefix = sensorFrame.robot().name();
    auto logWrenchWithoutGravityInFrame =
        [](const mc_rbdyn::RobotFrame & sensorFrame, const mc_rbdyn::RobotFrame & refFrame)
    {
      auto wrench = sensorFrame.wrench();
      const auto & fs = sensorFrame.forceSensor();
      const auto & X_0_fs = fs.X_0_f(sensorFrame.robot());
      auto X_0_refFrame = refFrame.position();
      auto X_fs_refFrame = X_0_refFrame * X_0_fs.inv();
      return X_fs_refFrame.dualMul(wrench);
    };

    // wrench with gravity in refAxisFrame
    auto logWrenchWithGravityInFrame =
        [](const mc_rbdyn::RobotFrame & sensorFrame, const mc_rbdyn::RobotFrame & refFrame)
    {
      auto X_0_f = refFrame.position();
      auto X_0_s = sensorFrame.forceSensor().X_0_f(sensorFrame.robot());
      auto X_s_f = X_0_f * X_0_s.inv();
      return X_s_f.dualMul(sensorFrame.forceSensor().wrench());
    };

    // Log in refFrame
    logger.addLogEntry(
        fmt::format("{}_{}-{}_without-gravity", log_prefix, sensorFrame.forceSensor().name(), refFrame.name()),
        [&sensorFrame, &refFrame, logWrenchWithoutGravityInFrame]()
        { return logWrenchWithoutGravityInFrame(sensorFrame, refFrame); });

    // Log in sensorFrame
    logger.addLogEntry(
        fmt::format("{}_{}-{}_without-gravity", log_prefix, sensorFrame.forceSensor().name(), sensorFrame.name()),
        [&sensorFrame, logWrenchWithoutGravityInFrame]()
        { return logWrenchWithoutGravityInFrame(sensorFrame, sensorFrame); });

    // Log in refFrame
    logger.addLogEntry(
        fmt::format("{}_{}-{}_with-gravity", log_prefix, sensorFrame.forceSensor().name(), refFrame.name()),
        [&sensorFrame, &refFrame, logWrenchWithGravityInFrame]()
        { return logWrenchWithGravityInFrame(sensorFrame, refFrame); });

    // Log in sensorFrame
    logger.addLogEntry(
        fmt::format("{}_{}-{}_with-gravity", log_prefix, sensorFrame.forceSensor().name(), sensorFrame.name()),
        [&sensorFrame, logWrenchWithGravityInFrame]()
        { return logWrenchWithGravityInFrame(sensorFrame, sensorFrame); });
  };

  // Log ATI sensor in sensor frame and refAxisFrame
  log_sensor(ctl.robot("panda_brace_femur").frame("BraceTopForceSensor"), *trajectory_.refAxisFrame());
  // Log Panda sensor in sensor frame and refAxisFrame
  log_sensor(ctl.robot("panda_brace_femur").frame("LeftHandForceSensor"), *trajectory_.refAxisFrame());

  // XXX somewhat hardcoded
  auto logForceShoeSensor = [&ctl, &logger, this](const std::string & forceSensorName, const std::string & logPrefix,
                                                  const std::string & datastoreEntry)
  {
    // In force sensor frame
    logger.addLogEntry(fmt::format("{}_sensorFrame", logPrefix),
                       [&ctl, datastoreEntry]()
                       {
                         if(ctl.datastore().has(datastoreEntry))
                         {
                           return ctl.datastore().get<sva::ForceVecd>(datastoreEntry);
                         }
                         return sva::ForceVecd::Zero();
                       });
    // In refAxis frame
    auto refAxisWrench = [this, &ctl, datastoreEntry, forceSensorName]()
    {
      if(ctl.datastore().has(datastoreEntry))
      {
        auto wrench = ctl.datastore().get<sva::ForceVecd>(datastoreEntry);
        auto & tibiaRobot = ctl.robot("brace_bottom_setup");
        const auto & fs = tibiaRobot.forceSensor(forceSensorName);
        const auto & X_0_fs = fs.X_0_f(tibiaRobot);
        auto X_0_tibia = trajectory_.refAxisFrame()->position();
        auto X_fs_tibia = X_0_tibia * X_0_fs.inv();
        return X_fs_tibia.dualMul(wrench);
      }
      return sva::ForceVecd::Zero();
    };
    logger.addLogEntry(fmt::format("{}_{}Frame", logPrefix, trajectory_.refAxisFrame()->name()), refAxisWrench);
    logger.addLogEntry(fmt::format("{}_{}Frame_norm", logPrefix, trajectory_.refAxisFrame()->name()),
                       [refAxisWrench]() { return refAxisWrench().force().norm(); });
  };
  logForceShoeSensor("BraceBottomForceSensor", "brace_bottom_setup_Bottom_raw", "ForceShoePlugin::RFForce");
  logForceShoeSensor("BraceBottomForceSensor", "brace_bottom_setup_Bottom_filtered", "ForceShoePlugin::RFfiltered");
  logForceShoeSensor("BraceTopForceSensor", "brace_bottom_setup_Top_raw", "ForceShoePlugin::LBForce");
  logForceShoeSensor("BraceTopForceSensor", "brace_bottom_setup_Top_filtered", "ForceShoePlugin::LBfiltered");

  logger.addLogEntry(fmt::format("transform_control_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()), [this]()
                     { return trajectory_.frame()->position() * trajectory_.refAxisFrame()->position().inv(); });
  logger.addLogEntry(fmt::format("transform_real_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()),
                     [this]()
                     {
                       auto & frame = *trajectory_.frame();
                       return ctl_.realRobot(frame.robot().name()).frame(frame.name()).position()
                              * trajectory_.refAxisFrame()->position().inv();
                     });
  logger.addLogEntry(fmt::format("rpy_control_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()),
                     [this]()
                     {
                       auto X_refFrame_Frame =
                           trajectory_.frame()->position() * trajectory_.refAxisFrame()->position().inv();
                       return mc_rbdyn::rpyFromMat(X_refFrame_Frame.rotation().inverse());
                     });

  logger.addLogEntry(fmt::format("rpy_control_{}_Link6", trajectory_.refAxisFrame()->name()),
                     [this]()
                     {
                       auto & frame = trajectory_.frame();
                       auto X_brace_bottom_setup_Link6 = ctl_.realRobot("brace_bottom_setup").frame("Link6").position();
                       auto X_PandaBraceFemur_Link2 = ctl_.realRobot("panda_brace_femur").frame("Link2").position();
                       auto X_refFrame_Frame = X_brace_bottom_setup_Link6 * X_PandaBraceFemur_Link2.inv();
                       auto rpy = mc_rbdyn::rpyFromMat(X_refFrame_Frame.rotation().transpose());
                       return rpy;
                     });

  logger.addLogEntry(fmt::format("rpy_real_{}_{}", trajectory_.refAxisFrame()->name(), frame->name()),
                     [this]()
                     {
                       auto & frame = *trajectory_.frame();
                       auto X_refFrame_Frame = ctl_.realRobot(frame.robot().name()).frame(frame.name()).position()
                                               * trajectory_.refAxisFrame()->position().inv();
                       return mc_rbdyn::rpyFromMat(X_refFrame_Frame.rotation().inverse());
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

void TrajectoryPlayer::addToGUI(mc_control::fsm::Controller & ctl, std::vector<std::string> category)
{
  auto & gui = *ctl.gui();
  category.push_back(trajectory_.name());
  gui.addElement(this, category, mc_rtc::gui::Input("Pause", pause_), mc_rtc::gui::Input("Track Force", trackForce_),
                 mc_rtc::gui::Input("Apply force when paused", applyForceWhenPaused_),
                 mc_rtc::gui::Input("Force Scaling (ratio)", forceScaling_));
  gui.addElement(
      this, category,
      mc_rtc::gui::Form(
          "Start Logging",
          [&ctl, this](const mc_rtc::Configuration & data)
          {
            auto filename = fmt::format("{}-LP_{:.3f}-RP_{:.3f}-{}", data("Patient Name"),
                                        static_cast<double>(data("Left Pressure")),
                                        static_cast<double>(data("Right Pressure")), static_cast<int>(data("Trial")));
            log_ = std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED,
                                                    static_cast<std::string>(ctl_.config()("results_dir")),
                                                    "trajectory-player");
            log_->start(filename, ctl_.timeStep, true);
            addToLogger(ctl, *log_);
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

  gui.addElement(this, category,
                 mc_rtc::gui::ArrayLabel("Real Femur Rotation (Tibia Frame)",
                                         [this]() -> Eigen::Vector3d
                                         {
                                           auto & frame = *trajectory_.frame();
                                           auto X_refFrame_Frame =
                                               ctl_.realRobot(frame.robot().name()).frame(frame.name()).position()
                                               * trajectory_.refAxisFrame()->position().inv();
                                           return mc_rbdyn::rpyFromMat(X_refFrame_Frame.rotation().inverse()) * 180
                                                  / mc_rtc::constants::PI;
                                         }),
                 mc_rtc::gui::ArrayLabel("Real Femur Translation (Tibia Frame)",
                                         [this]() -> Eigen::Vector3d
                                         {
                                           auto & frame = *trajectory_.frame();
                                           return (ctl_.realRobot(frame.robot().name()).frame(frame.name()).position()
                                                   * trajectory_.refAxisFrame()->position().inv())
                                               .translation();
                                         }));
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
