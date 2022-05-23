#include "GoRelative.h"

#include <mc_control/fsm/Controller.h>

void GoRelative::start(mc_control::fsm::Controller & ctl)
{
  if(config_.has("target"))
  {
    target_ = config_("target");
    has_target_ = true;
  }
  config_("robot", robot_);
  config_("frame", frame_);
  config_("stiffness", stiffness_);
  config_("weight", weight_);

  if(config_.has("completion"))
  {
    criteria_config_.load(config_("completion"));
  }
  if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), robot_);
  }
  auto & r = ctl.robot(robot_);
  if(!r.hasFrame(frame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No frame named {} in robot {}", name(), frame_, robot_);
  }

  transformTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robot_).frame(frame_), stiffness_, weight_);
  transformTask_->reset();
  ctl.solver().addTask(transformTask_);

  criteria_.configure(*transformTask_, ctl.timeStep, criteria_config_);

  if(has_target_)
  {
    transformTask_->target(target_ * r.posW());
  }

  output("OK");
}

bool GoRelative::run(mc_control::fsm::Controller & ctl)
{
  return criteria_.completed(*transformTask_);
}

void GoRelative::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(transformTask_);
}

EXPORT_SINGLE_STATE("GoRelative", GoRelative)
