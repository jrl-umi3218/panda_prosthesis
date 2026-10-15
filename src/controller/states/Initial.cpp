#include "Initial.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>
#include <utils.h>

namespace fs = std::filesystem;

void Initial::load(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[{}] Loading configuration from {}", name(), etc_file_);
  auto & robot = ctl.robot(robotName_);
  auto & realRobot = ctl.realRobot(robotName_);
  initial_pose_ = robot.posW();

  auto mbcToActuated = [&robot](const std::vector<std::vector<double>> & mbcJoints)
  {
    Eigen::VectorXd posture(robot.refJointOrder().size());
    for(int i = 0; i < robot.refJointOrder().size(); i++)
    {
      posture(i) = mbcJoints[robot.jointIndexInMBC(i)][0];
    }
    return posture;
  };

  auto initial_posture = mbcToActuated(robot.mbc().q);
  std::vector<std::vector<double>> initial_joints;

  if(fs::exists(etc_file_))
  {
    mc_rtc::Configuration initial(etc_file_);
    if(loadPose_ && initial.has(robotName_) && initial(robotName_).has("pose"))
    {
      initial_pose_ = initial(robotName_)("pose");
      robot.posW(initial_pose_);
      realRobot.posW(initial_pose_);
    }

    if(useJoints_)
    {
      if(initial.has(robotName_) && initial(robotName_).has("joints"))
      {
        initial_joints = initial(robotName_)("joints");
      }
      else
      {
        mc_rtc::log::error_and_throw("[{}] No \"joints\" defined in {}", name(), etc_file_);
      }
      if(reset_mbc_)
      {
        robot.mbc().q = initial_joints;
        realRobot.mbc().q = initial_joints;
      }

      PostureInterpolator::TimedValueVector values;
      values.emplace_back(0.0, initial_posture);
      values.emplace_back(duration_, mbcToActuated(initial_joints));
      postureInterp_.values(values);
    }
  }
  else
  {
    mc_rtc::log::error_and_throw("[{}] File {} does not exist", name(), etc_file_);
  }
  auto qActual = robot.mbc().q;
  robot.forwardKinematics();
  realRobot.forwardKinematics();

  if(!ctl.datastore().has(frame_))
  {
    ctl.datastore().make<sva::PTransformd>(frame_, sva::PTransformd::Identity());
  }
  ctl.datastore().assign(frame_, robot.frame(frame_).position());

  // Restore current joint configuration if we are not updating the mbc
  if(!reset_mbc_)
  {
    robot.mbc().q = qActual;
    robot.forwardKinematics();
    realRobot.mbc().q = qActual;
    realRobot.forwardKinematics();
  }
}

void Initial::start(mc_control::fsm::Controller & ctl)
{
  robotName_ = config_("robot", ctl.robot().name());
  if(!ctl.hasRobot(robotName_))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named \"{}\" in this controller", name(), robotName_);
  }
  auto & robot = ctl.robot(robotName_);
  initial_pose_ = robot.posW();
  config_("load", load_);
  config_("loadPose", loadPose_);
  config_("frame", frame_);
  config_("reset_mbc", reset_mbc_);
  config_("category", category_);
  config_("duration", duration_);
  transformTask_ = std::make_shared<mc_tasks::TransformTask>(robot.frame(frame_), config_("transformTaskStiffness", 60),
                                                             config_("transformTaskWeight", 500));
  transformTask_->reset();
  useDefaultPose_ = config_.has("default_pose");
  if(useDefaultPose_)
  {
    default_pose_ = static_cast<sva::PTransformd>(config_("default_pose"));
    transformTask_->target(default_pose_);
  }

  useJoints_ = ctl.getPostureTask(robotName_) != nullptr;

  if(!robot.hasFrame(frame_))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named \"{}\" in robot \"{}\"", name(), frame_, robotName_);
  }

  auto etc_dir = get_or_create_dir("calibration");
  auto controllerName = ctl.datastore().get<std::string>("ControllerName");
  etc_file_ = fmt::format("{}/{}/initial_{}.yaml", etc_dir, controllerName, robot.module().parameters()[0]);
  if(fs::exists(etc_file_))
  {
    mc_rtc::log::info("[{}] Found existing user configuration file at {}", name(), etc_file_);
  }
  else
  {
    etc_file_ = fmt::format("{}/{}/initial_{}.yaml", static_cast<std::string>(ctl.config()("CALIB_DIR_INIT")),
                            controllerName, robot.module().parameters()[0]);
  }

  if(useJoints_)
  {
    saved_stiffness_ = ctl.getPostureTask(robotName_)->stiffness();
    ctl.getPostureTask(robotName_)->stiffness(config_("stiffness", 1.0));
  }

  if(load_)
  { // Load this robot initial stance from saved configuration
    load(ctl);
    if(useDefaultPose_)
    {
      transformTask_->target(default_pose_);
      transformTaskActive_ = true;
      ctl.solver().addTask(transformTask_);
    }
  }
  else
  { // Allow to manually define it
    if(useDefaultPose_)
    {
      transformTaskActive_ = true;
      ctl.solver().addTask(transformTask_);
    }
    if(config_("showInstructions", false))
    {
      ctl.gui()->addElement(
          this, {},
          mc_rtc::gui::Label("Manual Calibration Instructions",
                             [this]() -> std::string
                             {
                               return fmt::format(
                                   "You can manually define the initial stance for the robots:\n"
                                   "- by moving its floating base marker\n"
                                   "- by changing its posture in the global posture task.\n"
                                   "- by adding tasks to control the end effector\n"
                                   "\n"
                                   "The goal is to align the TibiaCalibration and FemurCalibration frames with the "
                                   "robot base in the desired configuration. See the Calibration tab for details.");
                             }));
    }
    ctl.gui()->addElement(this, {"Calibration", robotName_},
                          mc_rtc::gui::Transform(
                              "Robot base pose", [this]() -> const sva::PTransformd & { return initial_pose_; },
                              [this](const sva::PTransformd & p)
                              {
                                initial_pose_ = p;
                                pose_changed_ = true;
                              }),
                          mc_rtc::gui::Checkbox(
                              "Activate EF Transform Task", [this, &ctl]() { return transformTaskActive_; },
                              [this, &ctl]()
                              {
                                transformTaskActive_ = !transformTaskActive_;
                                if(transformTaskActive_)
                                {
                                  transformTask_->reset();
                                  ctl.solver().addTask(transformTask_);
                                }
                                else
                                {
                                  ctl.solver().removeTask(transformTask_);
                                }
                              }),
                          mc_rtc::gui::Button("Done", [this]() { done_ = true; }));
  }

  mc_rtc::log::success("[{}] started", name());
  output("OK");
}

bool Initial::run(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot(robotName_);
  if(pose_changed_)
  {
    pose_changed_ = false;
    robot.posW(initial_pose_);
    ctl.realRobot(robotName_).posW(initial_pose_);
  }

  if(!load_)
  {
    if(transformTaskActive_)
    { // always reset the posture task to the current posture when the transformTask_ is active
      // this ensures that when removing it the robot retains its posture, and allows moving the posture through
      // the posture task otherwise
      ctl.getPostureTask(robotName_)->reset();
    }
    if(transformTaskActive_)
    {
      return transformTask_->speed().norm() < 1e-4 && transformTask_->eval().norm() < 1e-3;
    }
    return true;
  }
  if(load_)
  {
    if(t_ >= duration_)
    {
      if(useJoints_)
      {
        auto & pt = *ctl.getPostureTask(robotName_);
        // bool postureDone = pt.speed().norm() < 1e-4 && pt.eval().norm() < 1e-3;
        bool transformDone =
            !useDefaultPose_ || (transformTask_->speed().norm() < 1e-4 && transformTask_->eval().norm() < 1e-3);
        // return postureDone && transformDone;
        // return transformDone;
        return true; // FIXME adaptative gains
      }
      else
      {
        return true;
      }
    }
    else if(useJoints_)
    {
      auto actuated_posture = postureInterp_.compute(t_);
      auto posture = ctl.getPostureTask(robotName_)->posture();
      for(size_t i = 0; i < robot.refJointOrder().size(); i++)
      {
        posture[robot.jointIndexInMBC(i)][0] = actuated_posture[i];
      }
      ctl.getPostureTask(robotName_)->posture(posture);
      if(useDefaultPose_)
      {
        transformTask_->target(default_pose_);
      }
      t_ += ctl.timeStep;
      return false;
    }
    else
    {
      return true;
    }
  }

  return done_;
}

void Initial::teardown(mc_control::fsm::Controller & ctl)
{
  if(useJoints_)
  {
    ctl.getPostureTask(robotName_)->stiffness(saved_stiffness_);
  }
  ctl.solver().removeTask(transformTask_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Initial", Initial)
