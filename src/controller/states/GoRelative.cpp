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
  config("surface", surface_);
  config("stiffness", stiffness_);
  config("weight", weight_);
}

void GoRelative::start(mc_control::fsm::Controller & ctl)
{
  if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), robot_);
  }
  auto & r = ctl.robot(robot_);
  if(!r.hasSurface(surface_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No surface named {}", name(), surface_);
  }

  surfaceTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>(surface_, ctl.robots(), r.robotIndex());
  surfaceTask_->reset();
  surfaceTask_->stiffness(stiffness_);
  surfaceTask_->weight(weight_);
  ctl.solver().addTask(surfaceTask_);
  
  if(has_target_)
  {
    surfaceTask_->target(target_ * r.posW());
  }
}

bool GoRelative::run(mc_control::fsm::Controller & ctl)
{
  return false;
}

void GoRelative::teardown(mc_control::fsm::Controller & ctl)
{
}

EXPORT_SINGLE_STATE("GoRelative", GoRelative)
