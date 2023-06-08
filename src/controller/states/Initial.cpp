#include "Initial.h"

#include <mc_control/fsm/Controller.h>
#include <boost/filesystem.hpp>

void Initial::load(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[{}] Loading configuration from {}", name(), etc_file_);
  auto & robot = ctl.robot(robot_);
  initial_pose_ = robot.posW();
  auto initial_joints = robot.mbc().q;
  if(boost::filesystem::exists(etc_file_))
  {
    mc_rtc::Configuration initial(etc_file_);
    if(initial.has(robot_) && initial(robot_).has("pose"))
    {
      initial_pose_ = initial(robot_)("pose");
    }
    else
    {
      mc_rtc::log::error_and_throw("[{}] No \"pose\" defined in {}", name(), etc_file_);
    }

    if(initial.has(robot_) && initial(robot_).has("joints"))
    {
      initial_joints = initial(robot_)("joints");
    }
    else
    {
      mc_rtc::log::error_and_throw("[{}] No \"joints\" defined in {}", name(), etc_file_);
    }
  }
  ctl.getPostureTask(robot_)->posture(initial_joints);
  auto qActual = robot.mbc().q;
  robot.mbc().q = initial_joints;
  robot.posW(initial_pose_);
  robot.forwardKinematics();

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
  if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named \"{}\" in this controller", name(), robot_);
  }

  if(!ctl.robot(robot_).hasFrame(frame_))
  {
    mc_rtc::log::error_and_throw("[{}] No frame named \"{}\" in robot \"{}\"", name(), frame_, robot_);
  }

  if(!ctl.config().has("ETC_DIR") && ctl.config()("ETC_DIR").empty())
  {
    mc_rtc::log::error_and_throw("[{}] No \"ETC_DIR\"  entry specified", name());
  }
  etc_file_ = static_cast<std::string>(ctl.config()("ETC_DIR")) + "/initial_" + robot_ + ".yaml";

  saved_stiffness_ = ctl.getPostureTask(robot_)->stiffness();
  ctl.getPostureTask(robot_)->stiffness(config_("stiffness", 1.0));

  if(load_)
  {
    load(ctl);
  }

  category_.push_back(robot_);
  ctl.gui()->addElement(this, category_,
                        mc_rtc::gui::Transform(
                            fmt::format("Initial pose ({})", robot_),
                            [this]() -> const sva::PTransformd & { return initial_pose_; },
                            [this](const sva::PTransformd & p)
                            {
                              initial_pose_ = p;
                              pose_changed_ = true;
                            }));

  mc_rtc::log::success("[{}] started", name());
}

bool Initial::run(mc_control::fsm::Controller & ctl)
{
  if(pose_changed_)
  {
    pose_changed_ = false;
    ctl.robot(robot_).posW(initial_pose_);
  }
  return true;
}

void Initial::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.getPostureTask(robot_)->stiffness(saved_stiffness_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Initial", Initial)
