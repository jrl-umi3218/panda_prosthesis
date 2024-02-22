#include "Initial.h"

#include <mc_control/fsm/Controller.h>
#include <RBDyn/MultiBodyConfig.h>
#include <boost/filesystem.hpp>
#include <Eigen/src/Core/Matrix.h>

void Initial::load(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[{}] Loading configuration from {}", name(), etc_file_);
  auto & robot = ctl.robot(robot_);
  auto & realRobot = ctl.realRobot(robot_);
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

  if(boost::filesystem::exists(etc_file_))
  {
    mc_rtc::Configuration initial(etc_file_);
    if(initial.has(robot_) && initial(robot_).has("pose"))
    {
      initial_pose_ = initial(robot_)("pose");
      robot.posW(initial_pose_);
      realRobot.posW(initial_pose_);
    }

    if(useJoints_)
    {
      if(initial.has(robot_) && initial(robot_).has("joints"))
      {
        initial_joints = initial(robot_)("joints");
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
  robot_ = config_("robot", ctl.robot().name());
  initial_pose_ = ctl.robot(robot_).posW();
  config_("load", load_);
  config_("frame", frame_);
  config_("reset_mbc", reset_mbc_);
  config_("category", category_);
  config_("duration", duration_);
  if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named \"{}\" in this controller", name(), robot_);
  }

  useJoints_ = ctl.getPostureTask(robot_) != nullptr;

  if(!ctl.robot(robot_).hasFrame(frame_))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named \"{}\" in robot \"{}\"", name(), frame_, robot_);
  }

  if(!ctl.config().has("ETC_DIR") && ctl.config()("ETC_DIR").empty())
  {
    mc_rtc::log::error_and_throw("[{}] No \"ETC_DIR\"  entry specified", name());
  }
  auto controllerName = ctl.datastore().get<std::string>("ControllerName");
  etc_file_ = static_cast<std::string>(ctl.config()("ETC_DIR")) + "/" + controllerName + "/initial_" + robot_ + ".yaml";

  if(useJoints_)
  {
    saved_stiffness_ = ctl.getPostureTask(robot_)->stiffness();
    ctl.getPostureTask(robot_)->stiffness(config_("stiffness", 1.0));
  }

  if(load_)
  { // Load this robot initial stance from saved configuration
    load(ctl);
  }
  else
  { // Allow to manually define it
    ctl.gui()->addElement(
        this, category_,
        mc_rtc::gui::Label("Instructions",
                           [this]() -> std::string
                           {
                             return fmt::format(
                                 "You can manually define the initial stance for the robot {}:\n- by moving its "
                                 "floating base marker\n- by changing its posture in the global posture task.",
                                 robot_);
                           }),
        mc_rtc::gui::Transform(
            fmt::format("Initial pose ({})", robot_), [this]() -> const sva::PTransformd & { return initial_pose_; },
            [this](const sva::PTransformd & p)
            {
              initial_pose_ = p;
              pose_changed_ = true;
            }),
        mc_rtc::gui::Button("Done", [this]() { done_ = true; }));
  }

  mc_rtc::log::success("[{}] started", name());
  output("OK");
}

bool Initial::run(mc_control::fsm::Controller & ctl)
{
  if(pose_changed_)
  {
    pose_changed_ = false;
    ctl.robot(robot_).posW(initial_pose_);
    ctl.realRobot(robot_).posW(initial_pose_);
  }

  if(load_)
  {
    if(t_ >= duration_)
    {
      if(useJoints_)
      {
        auto & pt = *ctl.getPostureTask(robot_);
        return pt.speed().norm() < 1e-4 && pt.eval().norm() < 1e-3;
      }
      else
      {
        return true;
      }
    }
    else if(useJoints_)
    {
      auto actuated_posture = postureInterp_.compute(t_);
      auto posture = ctl.getPostureTask(robot_)->posture();
      auto & robot = ctl.robot(robot_);
      for(size_t i = 0; i < robot.refJointOrder().size(); i++)
      {
        posture[robot.jointIndexInMBC(i)][0] = actuated_posture[i];
      }
      ctl.getPostureTask(robot_)->posture(posture);
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
    ctl.getPostureTask(robot_)->stiffness(saved_stiffness_);
  }
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Initial", Initial)
