#include "GoRelative.h"

#include <mc_control/fsm/Controller.h>

void GoRelative::configure(const mc_rtc::Configuration & config)
{
  if(config.has("target"))
  {
    target_ = config("target");
    has_target_ = true;
  }
  config("robot", robot_);
  config("frame", frame_);
  config("stiffness", stiffness_);
  config("weight", weight_);
  if(config.has("completion"))
  {
    criteria_config_.load(config("completion"));
  }
}

void GoRelative::start(mc_control::fsm::Controller & ctl)
{
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
