#include "Initial.h"

#include <mc_control/fsm/Controller.h>
#include <boost/filesystem.hpp>

void Initial::save(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[{}] Saving initial state to {}", name(), etc_file_);
  mc_rtc::Configuration initial(etc_file_);
  initial.add(robot_);
  initial(robot_).add("pose", initial_pose_);
  initial(robot_).add("joints", ctl.robot(robot_).mbc().q);
  initial.save(etc_file_);
}

void Initial::load(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[{}] Loading configuration from {}", name(), etc_file_);
  initial_pose_ = default_pose_;
  auto initial_joints = ctl.robot().mbc().q;
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
  ctl.robot(robot_).posW(initial_pose_);
  ctl.robot().forwardKinematics();
}

void Initial::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
  config("category", category_);
  config("default_pose", default_pose_);
}

void Initial::start(mc_control::fsm::Controller & ctl)
{
  if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named \"{}\" in this controller", name(), robot_);
  }

  if(!ctl.config().has("ETC_DIR") && ctl.config()("ETC_DIR").empty())
  {
    mc_rtc::log::error_and_throw("[{}] No \"ETC_DIR\"  entry specified", name());
  }
  etc_file_ = static_cast<std::string>(ctl.config()("ETC_DIR")) + "/initial_" + robot_ + ".yaml";

  load(ctl);

  category_.push_back(robot_);
  ctl.gui()->addElement(this, category_,
                        mc_rtc::gui::Button("Done",
                                            [this, &ctl]() {
                                              output("OK");
                                              save(ctl);
                                            }),
                        mc_rtc::gui::Transform(
                            fmt::format("Initial pose ({})", robot_),
                            [this]() -> const sva::PTransformd & { return initial_pose_; },
                            [this](const sva::PTransformd & p) {
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
  return output().size() != 0;
}

void Initial::teardown(mc_control::fsm::Controller & ctl)
{
  save(ctl);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Initial", Initial)
